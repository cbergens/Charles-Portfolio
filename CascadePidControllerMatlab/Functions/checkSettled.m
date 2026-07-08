function [settled_time, settled_step] = checkSettled(signal, setpoint, dist_step, delta, band, vel_thresh, dwell_steps)
% checkSettled — measure how long a 1-D response takes to settle after a disturbance.
%
% DEFINITION
%   Settling time = the time from the disturbance until the signal ENTERS a
%   tolerance band around the setpoint AND STAYS there. The word "stays" is the
%   whole point: a single in-band sample during an oscillation does NOT count,
%   because the response is just swinging through the setpoint on its way out
%   the other side.
%
%   To be called "settled", all three of these must be true at once, and stay
%   true for a while:
%     1. it is inside the band:            |signal - setpoint| < band
%     2. it is not just passing through:   |velocity|          < vel_thresh
%     3. conditions 1 and 2 hold for at least dwell_steps steps in a row
%
% INPUTS
%   signal       1xN logged response for ONE axis (e.g. posX_Log)
%   setpoint     scalar target the signal should return to (e.g. posX_Desired)
%   dist_step    step index at which the disturbance was applied (clock starts here)
%   delta        timestep in seconds (0.005 here -> 200 Hz)
%   band         position tolerance in metres (start with 0.05)
%   vel_thresh   velocity gate in m/s (start with 0.05)
%   dwell_steps  how many consecutive in-band steps are required
%                (set to ~1 dominant oscillation period; ~200 steps at ~1 Hz)
%
% OUTPUTS
%   settled_time  seconds from the disturbance to the moment it entered and held
%                 (NaN if it never settles within the logged data)
%   settled_step  the step index of that entry point (-1 if it never settles)
%
% IMPORTANT
%   This measures GUST recovery: the disturbance is removed and the signal
%   returns to the setpoint. Under a CONSTANT disturbance the signal settles to
%   an OFFSET instead, the band around the setpoint never closes, and this
%   correctly returns NaN -- use a steady-state-offset metric for that case.


    % Assume "never settled" until proven otherwise. If the loop below never
    % confirms a settle, these defaults are what gets returned.
    settled_time = NaN;
    settled_step = -1;

    % entry_step remembers WHEN the signal first entered the band on the current
    % unbroken streak. A value of -1 means "not currently inside the band".
    entry_step = -1;


    % Walk forward in time, starting at the instant the disturbance was applied.
    for step = dist_step : length(signal)

        % --- Estimate velocity by finite-differencing the logged position. ---
        % v[k] = (x[k] - x[k-1]) / delta. Guard the very first sample, where
        % there is no previous point to subtract.
        if step > 1
            velocity = (signal(step) - signal(step - 1)) / delta;
        else
            velocity = 0;
        end
        

        % --- Test both conditions: inside the band AND moving slowly. ---
        % abs() makes the band symmetric around the setpoint. The velocity gate
        % is what rejects a fast pass-THROUGH the setpoint during an oscillation.
        in_band = (abs(signal(step) - setpoint) < band) && ...
                  (abs(velocity)               < vel_thresh);

        if in_band

            % First in-band sample of a fresh streak: record where it entered.
            if entry_step < 0
                entry_step = step;
            end

            % How long has the current streak lasted? (step - entry_step + 1)
            % counts the samples from entry to now, including both ends.
            if (step - entry_step + 1) >= dwell_steps

                % It entered the band and then held for long enough -> settled.
                % Report the ENTRY step, not "now": the response was effectively
                % settled the moment it entered and then proved it stayed.
                settled_step = entry_step;
                settled_time = (entry_step - dist_step) * delta;
                return;
            end

        else

            % Left the band (or moving too fast): the streak is broken. Reset
            % and wait for the next clean entry.
            entry_step = -1;
        end

    end

    % If we reach here the signal never held the band for long enough, so the
    % NaN / -1 defaults set at the top are returned unchanged.
end
