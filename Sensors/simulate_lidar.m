
function [lidar_ranges, lidar_angles, hit_points, hit_mask] = ...
    simulate_lidar(room_width, room_height, obstacles, ...
    lidar_position, lidar_orientation_deg, ...
    lidar_max_range, lidar_angle_resolution)

% ==============================================================
% SIMULATE LIDAR
%
% INPUTS:
% room_width              : Room width (m)
% room_height             : Room height (m)
% obstacles               : [X, Y, Radius] for each obstacle
% lidar_position          : [X, Y] position of LiDAR
% lidar_orientation_deg   : LiDAR orientation in degrees
% lidar_max_range         : Maximum measurement range (m)
% lidar_angle_resolution  : Angular resolution (degrees)
%
% OUTPUTS:
% lidar_ranges : Distance measured by each beam (m)
% lidar_angles : Direction of each beam (degrees)
% hit_points   : Endpoint of each beam
% hit_mask     : 1 if an obstacle/wall was detected, otherwise 0
%
% ==============================================================


% ==============================================================
% CELL 1 : GENERATE LIDAR ANGLES
% ==============================================================

lidar_angles = 0:lidar_angle_resolution: ...
    (360 - lidar_angle_resolution);

number_of_beams = length(lidar_angles);


% ==============================================================
% CELL 2 : INITIALIZE OUTPUTS
% ==============================================================

lidar_ranges = lidar_max_range * ones(1, number_of_beams);

hit_points = zeros(number_of_beams, 2);

hit_mask = false(1, number_of_beams);


% ==============================================================
% CELL 3 : PROCESS EACH LIDAR BEAM
% ==============================================================

for beam_number = 1:number_of_beams

    % Beam direction in the world coordinate frame

    beam_angle = lidar_orientation_deg + ...
        lidar_angles(beam_number);

    direction_x = cosd(beam_angle);
    direction_y = sind(beam_angle);


    % Initially, nearest distance is maximum LiDAR range

    nearest_distance = lidar_max_range;


    % ==========================================================
    % CHECK 1 : ROOM WALL INTERSECTION
    % ==========================================================

    % Right wall

    if direction_x > 0

        distance = (room_width - lidar_position(1)) / direction_x;

        if distance >= 0 && distance <= nearest_distance
            nearest_distance = distance;
            hit_mask(beam_number) = true;
        end

    % Left wall

    elseif direction_x < 0

        distance = (0 - lidar_position(1)) / direction_x;

        if distance >= 0 && distance <= nearest_distance
            nearest_distance = distance;
            hit_mask(beam_number) = true;
        end

    end


    % Top wall

    if direction_y > 0

        distance = (room_height - lidar_position(2)) / direction_y;

        if distance >= 0 && distance <= nearest_distance
            nearest_distance = distance;
            hit_mask(beam_number) = true;
        end

    % Bottom wall

    elseif direction_y < 0

        distance = (0 - lidar_position(2)) / direction_y;

        if distance >= 0 && distance <= nearest_distance
            nearest_distance = distance;
            hit_mask(beam_number) = true;
        end

    end


    % ==========================================================
    % CHECK 2 : CIRCULAR OBSTACLE INTERSECTION
    % ==============================================================

    for obstacle_number = 1:size(obstacles, 1)

        obstacle_x = obstacles(obstacle_number, 1);
        obstacle_y = obstacles(obstacle_number, 2);
        obstacle_radius = obstacles(obstacle_number, 3);

        % Vector from obstacle centre to LiDAR

        offset_x = lidar_position(1) - obstacle_x;
        offset_y = lidar_position(2) - obstacle_y;


        % Quadratic equation coefficients
        %
        % t^2 + 2*b*t + c = 0

        b = offset_x * direction_x + ...
            offset_y * direction_y;

        c = offset_x^2 + offset_y^2 - obstacle_radius^2;


        % Discriminant

        discriminant = b^2 - c;


        % Beam intersects the circle

        if discriminant >= 0

            root_value = sqrt(discriminant);

            distance_1 = -b - root_value;
            distance_2 = -b + root_value;

            % Select nearest non-negative intersection

            if distance_1 >= 0
                distance = distance_1;

            elseif distance_2 >= 0
                distance = distance_2;

            else
                distance = inf;
            end


            % Update nearest detected distance

            if distance <= nearest_distance

                nearest_distance = distance;
                hit_mask(beam_number) = true;

            end

        end

    end


    % ==========================================================
    % CELL 4 : STORE MEASUREMENT
    % ==============================================================

    lidar_ranges(beam_number) = nearest_distance;

    hit_points(beam_number, 1) = ...
        lidar_position(1) + nearest_distance * direction_x;

    hit_points(beam_number, 2) = ...
        lidar_position(2) + nearest_distance * direction_y;

end

end
