function [returnCode, obj_vel] = getObjectVelocity(sim, obj)
    %GETOBJECTPOSITION Summary of this function goes here
    %   Detailed explanation goes here
    
    obj_vel=nan(1,3);
    try
        obj_vel = sim.getObjectVelocity(obj);
        obj_vel = cellfun(@double, obj_vel); % Convert each cell content to double
        returnCode = 1;
    catch
        returnCode = 0;
    end
 

end

