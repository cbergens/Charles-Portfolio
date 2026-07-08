classdef TstCfg
    %TSTCFG Summary of this class goes here
    %   Detailed explanation goes here

    properties (Constant)
        stabilityDwellSteps = 200;
        stabilityBand = 0.05;
        stabilityVelThresh = 0.05;
        disturbanceSteps = [550, 600];
        disturbanceForceXYZ = [3, 0, 0];
        disturbanceTorqueABG = [0, 0, 0];
    end
end