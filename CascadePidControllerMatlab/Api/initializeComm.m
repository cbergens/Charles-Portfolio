function [ret_status, sim, clientID] = initializeComm()
    %% Summary of this function goes here
    % This function initializes communication with the
    % CopelliSim.
    % ---------------- Arguments -----------------------
    % No arguments
    %
    % ----------------- Return Values ------------------
    % ret_status: 0 - connection successfull
    %             1 - failure to connect
    %
    % sim: object to the prototype file
    %
    % clientID: object to the simulation
  
    client = RemoteAPIClient();
    sim = client.require('sim');
     
%     sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
%     sim.simxFinish(-1); % just in case, close all opened connections
%     clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,1);
    
    clientID = 0;
    if (clientID > -1)
        disp('Connected to CopelliaSim')
        % enable the synchronous mode on the client:
        % sim.simxSynchronous(clientID,true);

        % start the simulation:
        %sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot);
        ret_status = 0;
    else
        ret_status = 1;
    end
    
    
end

