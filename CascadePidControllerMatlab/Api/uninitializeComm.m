function [returnCode] = uninitializeComm(sim, clientID)
    %UNINITIALIZECOMM Summary of this function goes here
    %   Detailed explanation goes here
    
    sim.stopSimulation();
    returnCode = 0;
end

