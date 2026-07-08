function [returnCode, obj_ori] = getObjectOrientation(sim, obj)
    %GETOBJECTPOSITION Summary of this function goes here
    %   Detailed explanation goes here
    
    obj_ori=nan(1,3);
    try
        obj_ori = sim.getObjectOrientation(obj);
        obj_ori = cellfun(@double, obj_ori); % Convert each cell content to double
        returnCode = 1;
    catch
        returnCode = 0;
    end
 

end