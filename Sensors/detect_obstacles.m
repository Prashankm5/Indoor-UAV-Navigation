
function [front_status, left_status, rear_status, right_status] = ...
    detect_obstacles(front_distance, left_distance, ...
    rear_distance, right_distance, ...
    uav_radius, safety_margin, warning_distance)

% ==============================================================
% DETECT OBSTACLE PROXIMITY
%
% INPUTS:
% Four directional LiDAR distances
% UAV radius
% Safety margin
% Warning distance
%
% OUTPUTS:
% Directional status:
% 'CLEAR', 'CAUTION', or 'BLOCKED'
%
% ==============================================================


% ==============================================================
% CELL 1 : CALCULATE REQUIRED SAFETY DISTANCE
% ==============================================================

safe_distance = uav_radius + safety_margin;


% ==============================================================
% CELL 2 : FRONT DIRECTION
% ==============================================================

if front_distance <= safe_distance

    front_status = 'BLOCKED';

elseif front_distance <= warning_distance

    front_status = 'CAUTION';

else

    front_status = 'CLEAR';

end


% ==============================================================
% CELL 3 : LEFT DIRECTION
% ==============================================================

if left_distance <= safe_distance

    left_status = 'BLOCKED';

elseif left_distance <= warning_distance

    left_status = 'CAUTION';

else

    left_status = 'CLEAR';

end


% ==============================================================
% CELL 4 : REAR DIRECTION
% ==============================================================

if rear_distance <= safe_distance

    rear_status = 'BLOCKED';

elseif rear_distance <= warning_distance

    rear_status = 'CAUTION';

else

    rear_status = 'CLEAR';

end


% ==============================================================
% CELL 5 : RIGHT DIRECTION
% ==============================================================

if right_distance <= safe_distance

    right_status = 'BLOCKED';

elseif right_distance <= warning_distance

    right_status = 'CAUTION';

else

    right_status = 'CLEAR';

end

end
