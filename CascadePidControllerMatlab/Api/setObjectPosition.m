function [returnCode] = setObjectPosition(sim, clientID, obj, pos)
    %GETOBJECTPOSITION Summary of this function goes here

    try
        sim.setObjectPosition(obj, pos);
        returnCode = 1;
    catch
        returnCode = 0;
    end
 
    
end
