function [returnCode, obj_Angularvel] = getObjectAngularVelocity(sim, obj)
    %GETOBJECTPOSITION Summary of this function goes here
    %   Detailed explanation goes here
    
    obj_Angularvel=nan(1,3);
    try
        [~,obj_Angularvel] = sim.getObjectVelocity(obj);
        obj_Angularvel = cellfun(@double, obj_Angularvel); % Convert each cell content to double
        returnCode = 1;
    catch
        returnCode = 0;
    end
 

end

