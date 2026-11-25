
function [detected_points, front_distance, left_distance, ...
    rear_distance, right_distance] = ...
    process_lidar_scan(lidar_ranges, lidar_angles, ...
    hit_points, hit_mask, lidar_max_range)

% ==============================================================
% PROCESS LIDAR SCAN
%
% INPUTS:
% lidar_ranges    : Range measurement of each beam
% lidar_angles    : Beam angles relative to LiDAR orientation
% hit_points      : World coordinates of beam endpoints
% hit_mask        : 1 if a surface was detected
% lidar_max_range : Maximum LiDAR range
%
% OUTPUTS:
% detected_points : Coordinates of detected surfaces
% front_distance  : Nearest surface in front sector
% left_distance   : Nearest surface in left sector
% rear_distance   : Nearest surface in rear sector
% right_distance  : Nearest surface in right sector
%
% ==============================================================


% ==============================================================
% CELL 1 : EXTRACT DETECTED POINTS
% ==============================================================

detected_points = hit_points(hit_mask, :);


% ==============================================================
% CELL 2 : DEFINE DIRECTIONAL SECTORS
% ==============================================================

front_sector = lidar_angles >= 315 | lidar_angles < 45;

left_sector = lidar_angles >= 45 & lidar_angles < 135;

rear_sector = lidar_angles >= 135 & lidar_angles < 225;

right_sector = lidar_angles >= 225 & lidar_angles < 315;


% ==============================================================
% CELL 3 : CALCULATE MINIMUM DISTANCE IN EACH SECTOR
% ==============================================================

front_distance = lidar_max_range;

left_distance = lidar_max_range;

rear_distance = lidar_max_range;

right_distance = lidar_max_range;


if any(front_sector)

    front_distance = min(lidar_ranges(front_sector));

end


if any(left_sector)

    left_distance = min(lidar_ranges(left_sector));

end


if any(rear_sector)

    rear_distance = min(lidar_ranges(rear_sector));

end


if any(right_sector)

    right_distance = min(lidar_ranges(right_sector));

end

end
