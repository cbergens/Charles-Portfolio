function [returnCode, obj_pos] = getObjectPosition(sim, clientID, obj, firstCall)
    %GETOBJECTPOSITION Summary of this function goes here
    %   Detailed explanation goes here
    
    obj_pos=nan(1,3);
    try
        obj_pos = sim.getObjectPosition(obj);
        obj_pos = cellfun(@double, obj_pos); % Convert each cell content to double
        returnCode = 1;
    catch
        returnCode = 0;
    end
 

end

