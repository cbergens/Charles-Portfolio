%% Workspace Setup
clc
clear all
close all
addpath("Api");

%% Initialize Communication with CoppeliaSim
[retStatus, sim, clientId] = initializeComm();

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
    [returnCode, quad]    = getObjectReference(sim, clientId, 'Quadricopter');
    [returnCode, quadPos] = getObjectPosition(sim, clientId, quad, 1);
    [returnCode, quadOri] = getObjectOrientation(sim, quad);
    [returnCode, quadVel] = getObjectVelocity(sim, quad);

    % Store a handle for each rotor
    rotorHandle = {};
    for i = 1:4
        rotorHandle{i} = sim.getScriptHandle(['/Quadricopter/Quadricopter_propeller_respondable', num2str(i)]);
    end

    %% Environment Constants
    m      = 0.12;
    g      = 9.81;
    kF = 0.01;        % rotor thrust coefficient kf: F = thrust * w^2 (N*s^2/rad^2)
    iZ     = 1.6326e-03;  % yaw moment of inertia (kg*m^2), from CoppeliaSim shape properties
    kT     = 0.002;       % reaction torque coefficient (N*m*s^2/rad^2), from Lua propeller script

    %% Simulation Parameters
    w = ones(4, 1);
    trialSteps = 1500;
    delta = 0.005;

    %% Setpoints (m, m/s, rad, rad/s)
    posX_Desired = 0.0;
    posY_Desired = 0.0;
    posZ_Desired = 1.0;

    % Desired steady-state velocity (position loop derivative setpoints)
    velX_Desired = 0;
    velY_Desired = 0;
    velZ_Desired = 0;

    % Desired attitude
    yaw_Desired = 0.8;
    yaw_D_Desired = 0;
    pitch_D_Desired = 0;
    roll_D_Desired = 0;

    % Maximum wind resistance angles
    maxWindAngleX = 0.3;
    maxWindAngleY = 0.3;

    %% Controller Gains

    % X position PID - outer loop: position error -> desired pitch angle
    % clampX limit integral contribution to maxWindAngleX (rad) of pitch
    posX_P_Gain   = 1.3;
    posX_I_Gain   = 0.0;
    clampX        = maxWindAngleX / posX_I_Gain;
    posX_D_Gain   = 3.2;

    % Y position PID - outer loop: position error → desired roll angle
    % clampX limit integral contribution to maxWindAngleY (rad) of pitch
    posY_P_Gain   = 1.3;
    posY_I_Gain   = 0.0;
    clampY        = maxWindAngleY / posY_I_Gain;
    posY_D_Gain   = 3.2;

    % Z position PID - altitude hold with gravity feedforward
    % clampZ limits integral contribution to prevent integral windup on
    % takeoff
    posZ_P_Gain = 65;
    posZ_I_Gain = 2;
    clampZ      = 5;
    posZ_D_Gain = 25;

    % Attitude PD - inner loop: angle error + angular rate damping -> control torque
    attAngle_P_Gain = 20;
    attAngle_D_Gain = 6.56;

    % Yaw PD - map yaw angle error -> desired desired angular acceleration
    yaw_P_Gain = 15;
    yaw_D_Gain = 13;

    %% Integral Accumulators
    posX_Integral = 0;
    posY_Integral = 0;
    posZ_Integral = 0;


    %% Angular Rate Low-Pass Filter (one-pole IIR / exponential moving average)
    % Inner-loop D amplifies high-frequency content in the raw rate — that's what caps attAngle_P.
    % Filtering removes the buzz but ADDS phase lag in the control band (lag = atan(w/w_c)),
    % which competes with the actuator lag for the same phase budget. The cutoff is the tradeoff knob.
    %
    % Parameterize by cutoff frequency, NOT alpha: alpha is dimensionless and sample-rate-dependent;
    % f_c is physical and directly comparable to the measured buzz frequency and the ~1 Hz control band.
    %   alpha = 1 - exp(-2*pi*f_c*dt)   (pole-matching);   f_c = inf  ->  alpha = 1  ->  filtering OFF
    % Keep f_c well above the control crossover to minimize the phase tax; lower it toward w_buzz to attenuate.
    rateFilt_Fc    = inf;                                  % cutoff (Hz). inf = OFF (proven baseline). Try 8-20 to engage.
    rateFilt_Alpha = 1 - exp(-2*pi*rateFilt_Fc*delta);     % derived from cutoff — do not tune directly
    roll_D_Filt  = 0;
    pitch_D_Filt = 0;
    yaw_D_Filt   = 0;
    %% Control Loop

    for step = 1:trialSteps

        % Activate X/Y setpoints after Z has settled
        if (step > 600)
            posX_Desired = 2;
            posY_Desired = 2;
        end

        % Read state from simulator
        [returnCode, quadVel]        = getObjectVelocity(sim, quad);
        [returnCode, quadPos]        = getObjectPosition(sim, clientId, quad, 1);
        [returnCode, quadOri]        = getObjectOrientation(sim, quad);
        [returnCode, quadAngularVel] = getObjectAngularVelocity(sim, quad);

        % Attitude state - all external reference frame
        % sim.getObjectOrientation returns XYZ Euler [alpha, beta, gamma]:
        %   index 1 = alpha = rotation around X = roll
        %   index 2 = beta  = rotation around Y = pitch
        %   index 3 = gamma = rotation around Z = yaw
        % NOTE: THESE INDICES CAN CHANGE FROM SIM TO SIM, ENSURE THEY MATCH
        % WHAT YOU USE (this file is relatively non-portable from
        % CoppeliaSim to other sim softwares)

        roll(step)  = quadOri(1, 1);
        pitch(step) = quadOri(1, 2);
        yaw(step)   = quadOri(1, 3);

        roll_D  = quadAngularVel(1, 1);
        pitch_D = quadAngularVel(1, 2);
        yaw_D   = quadAngularVel(1, 3);

        % Log raw angular rates for buzz-frequency (FFT) measurement
        rollRate_Log(step)  = roll_D;
        pitchRate_Log(step) = pitch_D;
        yawRate_Log(step)   = yaw_D;

        % Low-pass the angular rates before using them for rate damping
        roll_D_Filt  = roll_D_Filt  + rateFilt_Alpha * (roll_D  - roll_D_Filt);
        pitch_D_Filt = pitch_D_Filt + rateFilt_Alpha * (pitch_D - pitch_D_Filt);
        yaw_D_Filt   = yaw_D_Filt   + rateFilt_Alpha * (yaw_D   - yaw_D_Filt);
        
        % Position and velocity state
        posX_Log(step) = quadPos(1, 1);
        posY_Log(step) = quadPos(1, 2);
        posZ_Log(step) = quadPos(1, 3);
        velX_Log(step) = quadVel(1, 1);
        velY_Log(step) = quadVel(1, 2);
        velZ_Log(step) = quadVel(1, 3);

        % Position errors
        posX_Error = posX_Desired - quadPos(1, 1);
        posY_Error = posY_Desired - quadPos(1, 2);
        posZ_Error = posZ_Desired - quadPos(1, 3);

        % Velocity (derivative) errors
        posX_D_Error = velX_Desired - quadVel(1, 1);
        posY_D_Error = velY_Desired - quadVel(1, 2);
        posZ_D_Error = velZ_Desired - quadVel(1, 3);

        %% Rotate X and Y Errors into the drone's reference frame to account
        %  for differing yaws.
        posX_Error_Body =   (posX_Error * cos(yaw(step))) + (posY_Error * sin(yaw(step)));
        posY_Error_Body = - (posX_Error * sin(yaw(step))) + (posY_Error * cos(yaw(step)));
        posX_D_Error_Body = (posX_D_Error * cos(yaw(step))) + (posY_D_Error * sin(yaw(step)));
        posY_D_Error_Body = - (posX_D_Error * sin(yaw(step))) + (posY_D_Error * cos(yaw(step)));

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

        % Yaw errors
        yaw_Error   = yaw_Desired   - yaw(step);
        yaw_D_Error = yaw_D_Desired - yaw_D_Filt;

        % Z controller — PID + gravity feedforward
        uZ = posZ_P_Gain * posZ_Error + ...
            posZ_I_Gain * posZ_Integral(step) + ...
            posZ_D_Gain * posZ_D_Error + ...
            (m * g) / (4 * kF * cos(roll(step)) * cos(pitch(step)));

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
        pitch_D_Error = pitch_D_Desired     - pitch_D_Filt;
        roll_D_Error  = roll_D_Desired      - roll_D_Filt;

        uX = (pitch_Error * attAngle_P_Gain) + (pitch_D_Error * attAngle_D_Gain);
        uY = (roll_Error  * attAngle_P_Gain) + (roll_D_Error  * attAngle_D_Gain);

        yaw_D2_Des = (yaw_P_Gain * yaw_Error) + (yaw_D_Gain * yaw_D_Error);

        abs_uWz = (iZ / kT) * yaw_D2_Des;

        uWz = [abs_uWz, -abs_uWz];

        % Control allocation — maps [uZ, uX, uY, uWz] → individual rotor wSq
        % Rotor layout: 1=front-left (CW), 2=back-left (CCW), 3=back-right (CW), 4=front-right (CCW)
        prop1 = -uX + uY + uZ + uWz(2);
        prop2 =  uX + uY + uZ + uWz(1);
        prop3 =  uX - uY + uZ + uWz(2);
        prop4 = -uX - uY + uZ + uWz(1);

        wSq = [prop1; prop2; prop3; prop4];

        %% Log Data
        uX_Log(step)      = uX;
        uY_Log(step)      = uY;
        uZ_Log(step)      = uZ;
        wSq_Min_Log(step) = min(wSq);

        % Negative wSq is physically impossible for this drone (no reverse thrust) — clamp to 0
        for i = 1:4
            w(i) = sqrt(max(0, wSq(i)));
        end

        % Send rotor commands to CoppeliaSim
        for i = 1:4
            sim.setScriptSimulationParameter(rotorHandle{i}, 'particleVelocity', w(i));
        end

        sim.step();

    end


    %% Plots
    t = (1:trialSteps) * delta;

    % Position tracking
    figure(1); clf;
    subplot(3,1,1);
    plot(t, posX_Log, 'b', t, posX_Desired * ones(1,trialSteps), 'r--');
    ylabel('X (m)'); title('X Position'); legend('Actual','Setpoint'); grid on;
    subplot(3,1,2);
    plot(t, posY_Log, 'b', t, posY_Desired * ones(1,trialSteps), 'r--');
    ylabel('Y (m)'); title('Y Position'); legend('Actual','Setpoint'); grid on;
    subplot(3,1,3);
    plot(t, posZ_Log, 'b', t, posZ_Desired * ones(1,trialSteps), 'r--');
    xlabel('Time (s)'); ylabel('Z (m)'); title('Z Position'); legend('Actual','Setpoint'); grid on;
    sgtitle('Position Tracking');

    % Attitude: actual vs desired
    figure(2); clf;
    subplot(3,1,1);
    plot(t, pitch, 'b', t, pitch_Desired, 'r--');
    ylabel('rad'); title('Pitch'); legend('Actual','Desired'); grid on;
    subplot(3,1,2);
    plot(t, roll, 'b', t, roll_Desired, 'r--');
    ylabel('rad'); title('Roll'); legend('Actual','Desired'); grid on;
    subplot(3,1,3);
    plot(t, yaw, 'b', t, yaw_Desired * ones(1,trialSteps), 'r--');
    xlabel('Time (s)'); ylabel('rad'); title('Yaw'); legend('Actual','Desired'); grid on;
    sgtitle('Attitude: Actual vs Desired');

    % Control signals
    figure(3); clf;
    subplot(3,1,1);
    plot(t, uX_Log, 'b'); ylabel('uX'); title('uX'); grid on;
    subplot(3,1,2);
    plot(t, uY_Log, 'b'); ylabel('uY'); title('uY'); grid on;
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

    %% Buzz-frequency measurement — FFT of the raw pitch rate
    % A limit cycle is ~single-frequency: it appears as a spike above the control band.
    % That spike is w_buzz ~= the attitude-loop phase crossover (and thus the actuator-lag limit).
    Fs  = 1 / delta;                         % sample rate = 200 Hz
    win = 900:trialSteps;                    % steady window (post X/Y transient); adjust as needed
    seg = pitchRate_Log(win);
    seg = seg - mean(seg);                   % remove DC so the buzz isn't buried in the 0-Hz bin
    N   = numel(seg);
    amp = abs(fft(seg)) / N;                 % two-sided amplitude spectrum
    f   = (0:N-1) * (Fs / N);                % frequency axis (Hz)
    half = 2:floor(N/2);                     % single-sided, skip DC bin

    figure(5); clf;
    plot(f(half), 2*amp(half), 'b'); grid on;
    xlabel('Frequency (Hz)'); ylabel('|pitch rate| (rad/s)');
    title('Pitch-rate spectrum — peak above the control band = \omega_{buzz}');
    xlim([0 Fs/2]);

    [~, kPk] = max(2*amp(half));
    fBuzz    = f(half(kPk));
    fControl = 1;                            % approx control-loop bandwidth (Hz)
    fcBal    = sqrt(fControl * fBuzz);       % log-midpoint cutoff: balances attenuation vs phase lag
    fprintf('Dominant pitch-rate frequency (w_buzz): %.2f Hz\n', fBuzz);
    fprintf('Balanced LPF cutoff f_c = %.2f Hz  (alpha = %.3f)\n', ...
            fcBal, 1 - exp(-2*pi*fcBal*delta));
    fprintf('  phase lag that cutoff adds at the %.0f Hz control band: %.1f deg\n', ...
            fControl, atan2(fControl, fcBal)*180/pi);

    uninitializeComm(sim, clientId);

else
    disp('Unable to connect to CoppeliaSim')
end
