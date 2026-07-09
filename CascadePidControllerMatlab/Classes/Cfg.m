classdef Cfg
% Cfg — central configuration for the XYZ cascade controller.
%
% All tuning constants, physical parameters, setpoints, and gains live here so
% there is a single source of truth. Access them as  Cfg.<name>  e.g. Cfg.delta.
%
% These are Constant properties: they CANNOT be reassigned at runtime
% (Cfg.delta = 1 throws an error). To change a value, edit it in this file.
% That is the point — one place to look, no stray magic numbers in the loop.
%
% NOTE: classdef must live in its own file named after the class (Cfg.m), on
% the MATLAB path. Add this folder with addpath() the same way you add Api.

    properties (Constant)

        %% --- Environment / physical constants (fixed by the airframe + sim) ---
        m   = 0.12;         % mass (kg)
        g   = 9.81;         % gravity (m/s^2)
        kF  = 0.01;         % rotor thrust coeff: F = kF * w^2  (N*s^2/rad^2)
        iX  = 1.6326e-03;   % roll moment of inertia 
        iY  = 1.6326e-03;   % pitch moment of inertia
        iZ  = 3.1693e-03;   % yaw moment of inertia (kg*m^2), from sim shape props
        kT  = 0.002;        % reaction torque coeff (N*m*s^2/rad^2), from Lua script
        L_X_Pos = 0.0919;   % front arm (-> pitch column
        L_X_Neg = 0.0919;   % back arm
        L_Y_Pos = 0.0919;   % left/right arm (-> roll column)
        L_Y_Neg = 0.0919;
        
        % Rotor layout: 1=front-lefDt (CW), 2=back-left (CCW), 3=back-right
        % (CW), 4=front-right (CCW)
        % Columns (control inputs): [tau_X (roll), tau_Y (pitch), uZ (thrust), uWz (yaw)]
        % Rows = rotors 1..4.
        
        controlAllocation = [Cfg.kF * Cfg.L_Y_Pos,  Cfg.kF * Cfg.L_Y_Pos, -Cfg.kF * Cfg.L_Y_Neg, -Cfg.kF * Cfg.L_Y_Neg; % roll
                            -Cfg.kF * Cfg.L_X_Pos,  Cfg.kF * Cfg.L_X_Neg,  Cfg.kF * Cfg.L_X_Neg, -Cfg.kF * Cfg.L_X_Pos; % pitch
                             Cfg.kF,                Cfg.kF,                Cfg.kF,                Cfg.kF              ; % thrust
                            -Cfg.kT,                Cfg.kT,               -Cfg.kT,                Cfg.kT              ] % yaw

        mixer = inv(Cfg.controlAllocation);

        % --- Simulation parameters ---
        trialSteps = 1500;   % number of control-loop iterations per run
        delta      = 0.005;  % timestep (s) -> 200 Hz control rate
        
        % --- Setpoints (m, m/s, rad/s) ---
        % Defaults for a hover test. Change these to define a different run.
        posX_Desired = 0.0;
        posY_Desired = 0.0;
        posZ_Desired = 1.0;

        % Desired steady-state velocity (position-loop derivative setpoints)
        velX_Desired = 0;
        velY_Desired = 0;
        velZ_Desired = 0;

        % Desired attitude rates (yaw-RATE control holds heading steady)
        yaw_Desired = 0;
        yaw_D_Desired = 0;
        pitch_D_Desired = 0;
        roll_D_Desired  = 0;

        % Maximum wind-resistance tilt angles (rad) — caps the integral authority
        maxWindAngleX = 0.3;
        maxWindAngleY = 0.3;

        % --- X position PID (outer loop: position error -> desired pitch) ---
        posX_P_Gain = 0.7;
        posX_I_Gain = 0.02;
        posX_D_Gain = 3.3;

        % --- Y position PID (outer loop: position error -> desired roll) ---
        posY_P_Gain = 0.7;
        posY_I_Gain = 0.02;
        posY_D_Gain = 3.3;

        % --- Z position PID (altitude hold with gravity feedforward) ---
        posZ_P_Gain = 0.5;
        posZ_I_Gain = 0.02;
        posZ_D_Gain = 0.7;
        clampZ      = 5;     % integral clamp (m*s) — prevents takeoff windup

        % --- Attitude PD (inner loop: angle error + rate damping -> torque) ---
        attAngle_P_Gain = 49;
        attAngle_D_Gain = 14;

        % --- Yaw rate control (regulate yaw rate to zero) ---
        yaw_P_Gain = 4.85;
        yaw_D_Gain = 5.85;
        
        % --- Derived integral clamps -------------------------------------------
        % Limit the integral's contribution to maxWindAngle (rad) of tilt:
        %   clamp = maxWindAngle / I_Gain.
        % Defined in terms of the constants above so they stay consistent if you
        % retune maxWindAngle or the I gains.
        clampX = Cfg.maxWindAngleX / Cfg.posX_I_Gain;   % = 0.3 / 0.02 = 15
        clampY = Cfg.maxWindAngleY / Cfg.posY_I_Gain;   % = 0.3 / 0.02 = 15

    end

end
