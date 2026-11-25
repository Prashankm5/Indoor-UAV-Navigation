
% ==============================================================
% TEST : A* GLOBAL PATH + PD CONTROLLER + UAV DYNAMICS
% STAGE 7 : WAYPOINT-BASED PATH TRACKING
% ==============================================================

clear;
clc;
close all;

project_folder = pwd;
addpath(genpath(project_folder));


% ==============================================================
% STEP 1: ENVIRONMENT PARAMETERS
% ==============================================================

room_width = 10.0;
room_height = 10.0;

grid_resolution = 0.10;

uav_radius = 0.25;
safety_margin = 0.15;

obstacles = [
    3.0, 3.0, 0.40;
    6.5, 4.0, 0.50;
    5.0, 7.0, 0.35
];

start_position = [1.5, 1.5];

goal_position = [8.5, 8.5];


% ==============================================================
% STEP 2: CREATE OCCUPANCY GRID
% ==============================================================

[occupancy_grid, grid_x, grid_y] = ...
    create_occupancy_grid(room_width, room_height, ...
    grid_resolution, uav_radius, safety_margin, obstacles);


% ==============================================================
% STEP 3: GENERATE GLOBAL PATH USING A*
% ==============================================================

[path, path_success, expanded_nodes, path_length] = ...
    astar(occupancy_grid, grid_x, grid_y, ...
    start_position, goal_position);


if ~path_success

    error('A* failed to generate a global path.');

end


number_of_path_points = size(path, 1);

fprintf('\n============================================\n');
fprintf(' A* GLOBAL PATH INFORMATION\n');
fprintf('============================================\n');

fprintf('Path found          : YES\n');
fprintf('Expanded nodes      : %d\n', expanded_nodes);
fprintf('Path points         : %d\n', number_of_path_points);
fprintf('Global path length  : %.3f m\n', path_length);

fprintf('============================================\n');


% ==============================================================
% STEP 4: INITIAL UAV STATE
% ==============================================================

position = start_position;

velocity = [0, 0];


% ==============================================================
% STEP 5: PD CONTROLLER PARAMETERS
% ==============================================================

Kp = 1.5;
Ki = 0;
Kd = 2.0;

integral_error = [0, 0];


% ==============================================================
% STEP 6: UAV DYNAMICS PARAMETERS
% ==============================================================

time_step = 0.05;

maximum_time = 120;

maximum_steps = round(maximum_time / time_step);

maximum_acceleration = 2.0;

maximum_speed = 1.0;


% ==============================================================
% STEP 7: WAYPOINT TRACKING PARAMETERS
% ==============================================================

waypoint_acceptance_radius = 0.1;

goal_speed_tolerance = 0.05;

current_waypoint_index = 2;

reached_goal = false;

collision_detected = false;


% ==============================================================
% STEP 8: INITIALISE TRAJECTORY STORAGE
% ==============================================================

trajectory = zeros(maximum_steps + 1, 2);

trajectory(1, :) = position;

time = zeros(maximum_steps + 1, 1);


% ==============================================================
% STEP 9: UAV PATH TRACKING LOOP
% ==============================================================

for step = 1:maximum_steps


    % ----------------------------------------------------------
    % 9.1: SELECT CURRENT TARGET WAYPOINT
    % ----------------------------------------------------------

    target_position = path(current_waypoint_index, :);


    % ----------------------------------------------------------
    % 9.2: CALCULATE PD CONTROL COMMAND
    % ----------------------------------------------------------

    [acceleration_command, integral_error] = ...
        pid_controller(target_position, position, ...
        velocity, integral_error, time_step, ...
        Kp, Ki, Kd);


    % ----------------------------------------------------------
    % 9.3: UPDATE UAV DYNAMICS
    % ----------------------------------------------------------

    [position, velocity, actual_acceleration] = ...
        update_uav(position, velocity, acceleration_command, ...
        time_step, maximum_acceleration, maximum_speed);


    % ----------------------------------------------------------
    % 9.4: STORE ACTUAL UAV TRAJECTORY
    % ----------------------------------------------------------

    trajectory(step + 1, :) = position;

    time(step + 1) = step * time_step;


    % ----------------------------------------------------------
    % 9.5: CHECK OCCUPANCY GRID COLLISION
    % ----------------------------------------------------------

    current_column = floor(position(1) / grid_resolution) + 1;

    current_row = floor(position(2) / grid_resolution) + 1;


    if current_row < 1 || current_row > size(occupancy_grid, 1) || ...
       current_column < 1 || current_column > size(occupancy_grid, 2)

        collision_detected = true;

        break;

    end


    if occupancy_grid(current_row, current_column) == 1

        collision_detected = true;

        break;

    end


    % ----------------------------------------------------------
    % 9.6: CALCULATE DISTANCE TO CURRENT WAYPOINT
    % ----------------------------------------------------------

    waypoint_error = norm(target_position - position);

    current_speed = norm(velocity);


    % ----------------------------------------------------------
    % 9.7: ADVANCE TO NEXT WAYPOINT
    % ----------------------------------------------------------

    if current_waypoint_index < number_of_path_points

        if waypoint_error <= waypoint_acceptance_radius

            current_waypoint_index = current_waypoint_index + 1;

        end

    else

        % ------------------------------------------------------
        % Final waypoint: check position and velocity together
        % ------------------------------------------------------

        if waypoint_error <= waypoint_acceptance_radius && ...
           current_speed <= goal_speed_tolerance

            reached_goal = true;

            break;

        end

    end

end


% ==============================================================
% STEP 10: REMOVE UNUSED STORAGE
% ==============================================================

trajectory = trajectory(1:step + 1, :);

time = time(1:step + 1);


% ==============================================================
% STEP 10A: CONTINUOUS TRAJECTORY COLLISION VALIDATION
% ==============================================================

collision_check_resolution = 0.025;

continuous_collision_detected = false;

collision_position = [NaN, NaN];

collision_segment = NaN;


for segment_number = 1:size(trajectory, 1)-1


    % Get consecutive UAV positions

    segment_start = trajectory(segment_number, :);

    segment_end = trajectory(segment_number + 1, :);


    % Calculate segment length

    segment_length = norm(segment_end - segment_start);


    % Determine number of samples

    number_of_samples = max( ...
        1, ceil(segment_length / collision_check_resolution));


    % Check intermediate positions along the segment

    for sample_number = 1:number_of_samples


        interpolation_factor = ...
            sample_number / number_of_samples;


        sample_position = ...
            segment_start + ...
            interpolation_factor * (segment_end - segment_start);


        % Convert position into occupancy grid indices

        sample_column = ...
            floor(sample_position(1) / grid_resolution) + 1;

        sample_row = ...
            floor(sample_position(2) / grid_resolution) + 1;


        % Check grid boundaries

        if sample_row < 1 || ...
           sample_row > size(occupancy_grid, 1) || ...
           sample_column < 1 || ...
           sample_column > size(occupancy_grid, 2)

            continuous_collision_detected = true;

            collision_position = sample_position;

            collision_segment = segment_number;

            break;

        end


        % Check occupied cells

        if occupancy_grid(sample_row, sample_column) == 1

            continuous_collision_detected = true;

            collision_position = sample_position;

            collision_segment = segment_number;

            break;

        end

    end


    if continuous_collision_detected

        break;

    end

end


% ==============================================================
% DISPLAY CONTINUOUS COLLISION VALIDATION
% ==============================================================

fprintf('\n============================================\n');
fprintf(' CONTINUOUS COLLISION VALIDATION\n');
fprintf('============================================\n');

fprintf('Sampling resolution : %.3f m\n', ...
    collision_check_resolution);

if continuous_collision_detected

    fprintf('Collision detected  : YES\n');

    fprintf('Trajectory segment  : %d\n', ...
        collision_segment);

    fprintf('Collision position  : [%.4f, %.4f] m\n', ...
        collision_position(1), collision_position(2));

else

    fprintf('Collision detected  : NO\n');

end

fprintf('============================================\n');

% ==============================================================
% STEP 11: CALCULATE FINAL PERFORMANCE VALUES
% ==============================================================

final_position_error = norm(goal_position - position);

final_speed = norm(velocity);

trajectory_differences = diff(trajectory, 1, 1);

trajectory_segment_lengths = sqrt( ...
    sum(trajectory_differences.^2, 2));

actual_trajectory_length = sum(trajectory_segment_lengths);


% ==============================================================
% STEP 12: DISPLAY RESULTS
% ==============================================================

fprintf('\n============================================\n');
fprintf(' A* PATH TRACKING TEST RESULTS\n');
fprintf('============================================\n');

fprintf('Initial position       : [%.3f, %.3f] m\n', ...
    start_position(1), start_position(2));

fprintf('Target position        : [%.3f, %.3f] m\n', ...
    goal_position(1), goal_position(2));

fprintf('Final position         : [%.3f, %.3f] m\n', ...
    position(1), position(2));

fprintf('Final position error   : %.5f m\n', ...
    final_position_error);

fprintf('Final speed            : %.5f m/s\n', ...
    final_speed);

fprintf('Simulation time        : %.2f s\n', ...
    time(end));

fprintf('Simulation steps       : %d\n', step);

fprintf('Global path length     : %.3f m\n', ...
    path_length);

fprintf('Actual trajectory length: %.3f m\n', ...
    actual_trajectory_length);

fprintf('Waypoints reached      : %d / %d\n', ...
    current_waypoint_index - 1, number_of_path_points);

if reached_goal

    fprintf('Goal reached           : YES\n');

else

    fprintf('Goal reached           : NO\n');

end

if collision_detected

    fprintf('Collision detected     : YES\n');

else

    fprintf('Collision detected     : NO\n');

end

fprintf('============================================\n');


if continuous_collision_detected

    fprintf('Continuous collision : YES\n');

else

    fprintf('Continuous collision : NO\n');

end
% ==============================================================
% ADDITIONAL DIAGNOSTICS : WAYPOINT TRACKING
% ==============================================================

current_target_position = path(current_waypoint_index, :);

distance_to_current_waypoint = ...
    norm(current_target_position - position);

distance_to_final_goal = ...
    norm(goal_position - position);


fprintf('\n============================================\n');
fprintf(' WAYPOINT TRACKING DIAGNOSTICS\n');
fprintf('============================================\n');

fprintf('Current waypoint index : %d / %d\n', ...
    current_waypoint_index, number_of_path_points);

fprintf('Current waypoint       : [%.4f, %.4f] m\n', ...
    current_target_position(1), current_target_position(2));

fprintf('Distance to waypoint   : %.5f m\n', ...
    distance_to_current_waypoint);

fprintf('Distance to final goal : %.5f m\n', ...
    distance_to_final_goal);

fprintf('Current UAV speed      : %.5f m/s\n', ...
    norm(velocity));

fprintf('============================================\n');

% ==============================================================
% ADDITIONAL DIAGNOSTICS : FINAL WAYPOINT BEHAVIOUR
% ==============================================================

current_error_vector = current_target_position - position;

velocity_towards_waypoint = ...
    dot(current_error_vector, velocity);


fprintf('\n============================================\n');
fprintf(' FINAL WAYPOINT MOVEMENT DIAGNOSTICS\n');
fprintf('============================================\n');

fprintf('Position error vector : [%.6f, %.6f] m\n', ...
    current_error_vector(1), current_error_vector(2));

fprintf('Velocity vector       : [%.6f, %.6f] m/s\n', ...
    velocity(1), velocity(2));

fprintf('Error-velocity dot product : %.8f\n', ...
    velocity_towards_waypoint);

fprintf('\nNext A* waypoints:\n');

first_waypoint = current_waypoint_index;

last_waypoint = min( ...
    current_waypoint_index + 4, number_of_path_points);

disp(path(first_waypoint:last_waypoint, :));

fprintf('============================================\n');


% ==============================================================
% DISPLAY LAST 10 TRAJECTORY POSITIONS
% ==============================================================

fprintf('\nLast 10 UAV trajectory positions:\n');

first_sample = max(1, size(trajectory, 1) - 9);

disp(trajectory(first_sample:end, :));
% ==============================================================
% STEP 13: PLOT GLOBAL PATH AND ACTUAL UAV TRAJECTORY
% ==============================================================

figure;

imagesc(grid_x, grid_y, occupancy_grid);

set(gca, 'YDir', 'normal');

colormap(flipud(gray));

hold on;


% Global A* path
h_path = plot(path(:, 1), path(:, 2), ...
    'c--', 'LineWidth', 1.5);

% Actual UAV trajectory
h_trajectory = plot(trajectory(:, 1), trajectory(:, 2), ...
    'b-', 'LineWidth', 2);

% Start
h_start = plot(start_position(1), start_position(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

% Goal
h_goal = plot(goal_position(1), goal_position(2), ...
    'r*', 'MarkerSize', 12, 'LineWidth', 2);

% Final UAV position
h_final = plot(position(1), position(2), ...
    'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);

legend([h_path, h_trajectory, h_start, h_goal, h_final], ...
    {'A* Global Path', 'Actual UAV Trajectory', ...
    'Start', 'Goal', 'Final UAV Position'}, ...
    'Location', 'bestoutside');

hold off;

% STEP 13: LIVE NAVIGATION ANIMATION

lidar_max_range = 5.0;
lidar_angle_resolution = 1.0;

animate_navigation(room_width, room_height, ...
    occupancy_grid, grid_x, grid_y, path, trajectory, ...
    start_position, goal_position, obstacles, ...
    lidar_max_range, lidar_angle_resolution);