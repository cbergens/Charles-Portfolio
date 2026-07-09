%% CURRENTLY RESOLVING ISSUE WITH ROTOR MOMENTS OF INTERTIA AND STEINER SHIFT TERMS

%% Workspace Setup
clc
clear all
close all
addpath("Api");
addpath("Functions");
addpath("Classes");

%% Initialize Communication with CoppeliaSim
[retStatus, sim, clientID] = initializeComm();

%% Reload scene from disk — guarantees clean initial state every run
scenePath = sim.getStringParam(sim.stringparam_scene_path_and_name);
sim.loadScene(scenePath);
pause(0.5);

%% Enable Stepping Mode
sim.setStepping(true);

%% Start Simulation
sim.startSimulation();

%% Main
if (retStatus == 0)

    % Reference the 'Quadricopter' object in CoppeliaSim as 'quad' in MATLAB
    [returnCode, quad]    = getObjectReference(sim, clientID, 'Quadricopter');
    [returnCode, quadPos] = getObjectPosition(sim, clientID, quad, 1);
    [returnCode, quadOri] = getObjectOrientation(sim, quad);
    [returnCode, quadVel] = getObjectVelocity(sim, quad);

    % Store a handle for each rotor
    rotorHandle = {};
    for i = 1:4
        rotorHandle{i} = sim.getScriptHandle(['/Quadricopter/Quadricopter_propeller_respondable', num2str(i)]);
    end

    % Get L for texting
    quadH = sim.getObject('/Quadricopter');
    for i = 1:4
        rotorH = sim.getObject(['/Quadricopter/Quadricopter_propeller_respondable', num2str(i)]);
        p = cellfun(@double, sim.getObjectPosition(rotorH, quadH));   % body-frame [x y z]
        fprintf('Rotor %d:  x=% .4f  y=% .4f  r=%.4f\n', i, p(1), p(2), hypot(p(1),p(2)));
    end

    %% Load configuration into locals (copied from Cfg once, before the loop)
    %  Reading Cfg.* here rather than inside the loop avoids per-iteration
    %  property-lookup overhead. To change a value, edit Cfg.m — not here.

    % Environment / physical constants
    m   = Cfg.m;
    g   = Cfg.g;
    kF  = Cfg.kF;
    iX  = Cfg.iX;
    iY  = Cfg.iY;
    iZ  = Cfg.iZ;
    kT  = Cfg.kT;
    mixer = Cfg.mixer;
    % Simulation parameters
    trialSteps = Cfg.trialSteps;
    delta      = Cfg.delta;

    %% Simulation Parameters (runtime state)
    w = ones(4, 1);
    testDisturbance = true;
    
    stabilityDwellSteps = TstCfg.stabilityDwellSteps;
    stabilityBand = TstCfg.stabilityBand;
    stabilityVelThresh = TstCfg.stabilityVelThresh;
    disturbanceSteps = TstCfg.disturbanceSteps;
    disturbanceForceXYZ = TstCfg.disturbanceForceXYZ;
    disturbanceTorqueABG = TstCfg.disturbanceTorqueABG;

    %% Setpoints (m, m/s, rad/s)
    posX_Desired = Cfg.posX_Desired;
    posY_Desired = Cfg.posY_Desired;
    posZ_Desired = Cfg.posZ_Desired;

    % Desired steady-state velocity (position loop derivative setpoints)
    velX_Desired = Cfg.velX_Desired;
    velY_Desired = Cfg.velY_Desired;
    velZ_Desired = Cfg.velZ_Desired;

    % Desired attitude rates
    yaw_Desired = Cfg.yaw_Desired;
    yaw_D_Desired = Cfg.yaw_D_Desired;   % yaw RATE control — hold heading steady
    pitch_D_Desired = Cfg.pitch_D_Desired;
    roll_D_Desired  = Cfg.roll_D_Desired;

    % Maximum wind resistance angles
    maxWindAngleX = Cfg.maxWindAngleX;
    maxWindAngleY = Cfg.maxWindAngleY;

    %% Controller Gains (from Cfg)

    % X position PID — outer loop: position error -> desired pitch angle
    posX_P_Gain = Cfg.posX_P_Gain;
    posX_I_Gain = Cfg.posX_I_Gain;
    posX_D_Gain = Cfg.posX_D_Gain;

    % Y position PID — outer loop: position error -> desired roll angle
    posY_P_Gain = Cfg.posY_P_Gain;
    posY_I_Gain = Cfg.posY_I_Gain;
    posY_D_Gain = Cfg.posY_D_Gain;

    % Z position PID — altitude hold with gravity feedforward
    posZ_P_Gain = Cfg.posZ_P_Gain;
    posZ_I_Gain = Cfg.posZ_I_Gain;
    posZ_D_Gain = Cfg.posZ_D_Gain;

    % Attitude PD — inner loop: angle error + angular rate damping -> control torque
    attAngle_P_Gain = Cfg.attAngle_P_Gain;
    attAngle_D_Gain = Cfg.attAngle_D_Gain;

    % Yaw rate control — regulate yaw rate to zero (heading-hold via rate damping)
    yaw_P_Gain = Cfg.yaw_P_Gain;
    yaw_D_Gain = Cfg.yaw_D_Gain;
    
    % Integral clamps — derived in Cfg as maxWindAngle / I_Gain
    clampX = Cfg.clampX;   % = 15
    clampY = Cfg.clampY;   % = 15
    clampZ = Cfg.clampZ;

    %% Preallocate logged arrays
    roll           = zeros(1, trialSteps);
    pitch          = zeros(1, trialSteps);
    yaw            = zeros(1, trialSteps);
    posX_Log       = zeros(1, trialSteps);
    posY_Log       = zeros(1, trialSteps);
    posZ_Log       = zeros(1, trialSteps);
    posX_Integral  = zeros(1, trialSteps);
    posY_Integral  = zeros(1, trialSteps);
    posZ_Integral  = zeros(1, trialSteps);
    yaw_Integral   = zeros(1, trialSteps);
    accelX_Desired = zeros(1, trialSteps);
    accelY_Desired = zeros(1, trialSteps);
    pitch_Desired  = zeros(1, trialSteps);
    roll_Desired   = zeros(1, trialSteps);
    tau_X_Log      = zeros(1, trialSteps);
    tau_Y_Log      = zeros(1, trialSteps);
    uZ_Log         = zeros(1, trialSteps);
    wSq_Min_Log    = zeros(1, trialSteps);

    %% Control Loop

    for step = 1:trialSteps
        
        % Disturb the drone for 100ms
        if (testDisturbance) && (disturbanceSteps(1) < step) && (step < disturbanceSteps(2)) 
            sim.addForceAndTorque(quad, disturbanceForceXYZ, disturbanceTorqueABG);
        end
       
        % Read state from simulator
        [returnCode, quadVel]        = getObjectVelocity(sim, quad);
        [returnCode, quadPos]        = getObjectPosition(sim, clientID, quad, 1);
        [returnCode, quadOri]        = getObjectOrientation(sim, quad);
        [returnCode, quadAngularVel] = getObjectAngularVelocity(sim, quad);

        % Attitude state — external reference frame
        % sim.getObjectOrientation returns XYZ Euler [alpha, beta, gamma]:
        %   index 1 = alpha = rotation around X = roll
        %   index 2 = beta  = rotation around Y = pitch
        %   index 3 = gamma = rotation around Z = yaw
        % NOTE: THESE INDICES CAN CHANGE FROM SIM TO SIM — verify against your scene.
        roll(step)  = quadOri(1, 1);
        pitch(step) = quadOri(1, 2);
        yaw(step)   = quadOri(1, 3);

        roll_D  = quadAngularVel(1, 1);
        pitch_D = quadAngularVel(1, 2);
        yaw_D   = quadAngularVel(1, 3);

        % Position state (logged for plots)
        posX_Log(step) = quadPos(1, 1);
        posY_Log(step) = quadPos(1, 2);
        posZ_Log(step) = quadPos(1, 3);

        % Position errors
        posX_Error = posX_Desired - quadPos(1, 1);
        posY_Error = posY_Desired - quadPos(1, 2);
        posZ_Error = posZ_Desired - quadPos(1, 3);

        % Velocity (derivative) errors
        posX_D_Error = velX_Desired - quadVel(1, 1);
        posY_D_Error = velY_Desired - quadVel(1, 2);
        posZ_D_Error = velZ_Desired - quadVel(1, 3);

        %% Rotate X and Y errors into the body frame to account for heading.
        %  With yaw-rate control the heading stays near 0, so this is ~identity,
        %  but keeping it makes the controller correct for any heading.
        posX_Error_Body =   (posX_Error * cos(yaw(step))) + (posY_Error * sin(yaw(step)));
        posY_Error_Body = - (posX_Error * sin(yaw(step))) + (posY_Error * cos(yaw(step)));
        posX_D_Error_Body = (posX_D_Error * cos(yaw(step))) + (posY_D_Error * sin(yaw(step)));
        posY_D_Error_Body = - (posX_D_Error * sin(yaw(step))) + (posY_D_Error * cos(yaw(step)));

        % Yaw rate error — drive yaw rate to the setpoint (0)
        yaw_D_Error = yaw_D_Desired - yaw_D;
        yaw_Error = yaw_Desired - yaw(step);

        % Integral errors — Riemann sum with dt == delta, clamped to prevent windup
        if (step == 1)
            posX_Integral(step) = posX_Error_Body * delta;
            posY_Integral(step) = posY_Error_Body * delta;
            posZ_Integral(step) = posZ_Error * delta;
        else
            posX_Integral(step) = posX_Integral(step - 1) + posX_Error_Body * delta;
            posY_Integral(step) = posY_Integral(step - 1) + posY_Error_Body * delta;
            posZ_Integral(step) = posZ_Integral(step - 1) + posZ_Error * delta;
        end

        posX_Integral(step) = max(-clampX, min(clampX, posX_Integral(step)));
        posY_Integral(step) = max(-clampY, min(clampY, posY_Integral(step)));
        posZ_Integral(step) = max(-clampZ, min(clampZ, posZ_Integral(step)));

        % Z controller — PID + gravity feedforward
        uZ = posZ_P_Gain * posZ_Error + ...
             posZ_I_Gain * posZ_Integral(step) + ...
             posZ_D_Gain * posZ_D_Error + ...
             (m * g) / (cos(roll(step)) * cos(pitch(step)));

        % Outer loop — position error -> desired acceleration -> desired tilt angle
        % Physics: tan(theta) = a/g -> theta = atan(a/g)
        accelX_Desired(step) = posX_P_Gain * posX_Error_Body + ...
                               posX_I_Gain * posX_Integral(step) + ...
                               posX_D_Gain * posX_D_Error_Body;

        accelY_Desired(step) = posY_P_Gain * posY_Error_Body + ...
                               posY_I_Gain * posY_Integral(step) + ...
                               posY_D_Gain * posY_D_Error_Body;

        pitch_Desired(step) =  atan2(accelX_Desired(step), g);
        roll_Desired(step)  = -atan2(accelY_Desired(step), g);

        % Inner attitude loop — angle error + angular rate damping
        pitch_Error   = pitch_Desired(step) - pitch(step);
        roll_Error    = roll_Desired(step)  - roll(step);
        pitch_D_Error = pitch_D_Desired     - pitch_D;
        roll_D_Error  = roll_D_Desired      - roll_D;

        roll_D2_Des = (roll_Error  * attAngle_P_Gain) + (roll_D_Error  * attAngle_D_Gain);
        pitch_D2_Des = (pitch_Error * attAngle_P_Gain) + (pitch_D_Error * attAngle_D_Gain);
        
        tau_X = iX * roll_D2_Des;
        tau_Y = iY * pitch_D2_Des;

        % Yaw rate controller — desired yaw angular acceleration -> rotor wSq differential
        yaw_D2_Des = (yaw_P_Gain * yaw_Error) + (yaw_D_Gain * yaw_D_Error);
        tau_Z = iZ * yaw_D2_Des;

        % Control allocation — maps ctrls -> individual rotor wSq
        % ctrls order: [tau_X (roll); tau_Y (pitch); uZ (thrust); uWz (yaw)]

        ctrls = [tau_X; tau_Y; uZ; tau_Z];
        wSq = mixer * ctrls;

        %% Log Data
        tau_X_Log(step)   = tau_X;
        tau_Y_Log(step)   = tau_Y;
        uZ_Log(step)      = uZ;
        wSq_Min_Log(step) = min(wSq);

        % Negative wSq is physically impossible (no reverse thrust) — clamp to 0
        for i = 1:4
            w(i) = sqrt(max(0, wSq(i)));
        end

        % Send rotor commands to CoppeliaSim
        for i = 1:4
            sim.setScriptSimulationParameter(rotorHandle{i}, 'particleVelocity', w(i));
        end

        sim.step();

    end
    
    % Check if the drone has settled since disturbance
        [timeSettled, stepSettled] = checkSettled(posX_Log, ...
                                                  posX_Desired, ...
                                                  disturbanceSteps(2), ...
                                                  delta, ...
                                                  stabilityBand, ...
                                                  400, ...
                                                  stabilityDwellSteps);

    %% Plots
    t = (1:trialSteps) * delta;

    % Position tracking
    figure(1); clf;
    subplot(3,1,1);
    plot(t, posX_Log, 'b', t, posX_Desired * ones(1,trialSteps), 'r--');
    xline(stepSettled * delta, 'g--', sprintf('t_s = %.2f s', timeSettled));
    yline(posX_Desired + stabilityBand, 'k:');
    yline(posX_Desired - stabilityBand, 'k:');
    yline(max(posX_Log), 'y--', sprintf('Maximum Displacement: %.2f', max(posX_Log)))
    yline(min(posX_Log), 'y--', sprintf('Minimum Displacement: %.2f', min(posX_Log)))
    ylabel('X (m)'); title('X Position'); legend('Actual','Setpoint'); grid on;
    subplot(3,1,2);
    plot(t, posY_Log, 'b', t, posY_Desired * ones(1,trialSteps), 'r--');
    ylabel('Y (m)'); title('Y Position'); legend('Actual','Setpoint'); grid on;
    subplot(3,1,3);
    plot(t, posZ_Log, 'b', t, posZ_Desired * ones(1,trialSteps), 'r--');
    xlabel('Time (s)'); ylabel('Z (m)'); title('Z Position'); legend('Actual','Setpoint'); grid on;
    sgtitle('Position Tracking');

    % Attitude
    figure(2); clf;
    subplot(3,1,1);
    plot(t, pitch, 'b', t, pitch_Desired, 'r--');
    ylabel('rad'); title('Pitch'); legend('Actual','Desired'); grid on;
    subplot(3,1,2);
    plot(t, roll, 'b', t, roll_Desired, 'r--');
    ylabel('rad'); title('Roll'); legend('Actual','Desired'); grid on;
    subplot(3,1,3);
    plot(t, yaw, 'b', t, zeros(1,trialSteps), 'r--');
    xlabel('Time (s)'); ylabel('rad'); title('Yaw Angle (held steady by rate control)'); legend('Actual','Setpoint'); grid on;
    sgtitle('Attitude');

    % Control signals
    figure(3); clf;
    subplot(3,1,1);
    plot(t, tau_X_Log, 'b'); ylabel('\tau_X (roll)'); title('\tau_X — roll torque'); grid on;
    subplot(3,1,2);
    plot(t, tau_Y_Log, 'b'); ylabel('\tau_Y (pitch)'); title('\tau_Y — pitch torque'); grid on;
    subplot(3,1,3);
    plot(t, uZ_Log, 'b'); xlabel('Time (s)'); ylabel('uZ'); title('uZ (thrust)'); grid on;
    sgtitle('Control Signals');

    % Outer loop diagnostics
    figure(4); clf;
    subplot(3,1,1);
    plot(t, accelX_Desired, 'b', t, accelY_Desired, 'r');
    ylabel('m/s^2'); title('Desired Accelerations'); legend('X','Y'); grid on;
    subplot(3,1,2);
    plot(t, posX_Integral, 'b', t, posY_Integral, 'r', t, posZ_Integral, 'g');
    ylabel('m*s'); title('Position Integrals'); legend('X','Y','Z'); grid on;
    subplot(3,1,3);
    plot(t, wSq_Min_Log, 'b'); yline(0, 'r--');
    xlabel('Time (s)'); ylabel('rad^2/s^2'); title('Min Rotor wSq (below 0 = clamped)'); grid on;
    sgtitle('Outer Loop Diagnostics');

    uninitializeComm(sim, clientID);

else
    disp('Unable to connect to CoppeliaSim')
end
