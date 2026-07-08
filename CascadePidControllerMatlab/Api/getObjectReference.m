function [returnCode, obj_ref] = getObjectReference(sim, clientID, objectName)
    %% Summary of this function goes here
    %   This fucntion will return the reference of the 
    if ~startsWith(objectName, '/')
        objectName = strcat('/', objectName);
    end
    obj_ref = nan;
    try
        obj_ref= sim.getObject(objectName);
        returnCode = 1;
    catch
        returnCode = 0;
    end
    
%     sim.getObject('/Quadricopter')
end

