
% ==============================================================
% MAIN : INDOOR UAV NAVIGATION
% STAGE 9 : COUPLED UAV + LIDAR + CONTROLLER SIMULATION
% APPROACH A : KNOWN MAP
% ==============================================================

clear;
clc;
close all;


% STEP 1: ENVIRONMENT PARAMETERS

room_width = 10;
room_height = 10;

grid_resolution = 0.1;

uav_radius = 0.25;
safety_margin = 0.15;

obstacles = [
    3.0, 3.0, 0.40;
    6.5, 4.0, 0.50;
    5.0, 7.0, 0.35
];

start_position = [1.5, 1.5];
goal_position = [8.5, 8.5];


% STEP 2: CREATE OCCUPANCY GRID

[occupancy_grid, grid_x, grid_y] = ...
    create_occupancy_grid(room_width, room_height, ...
    grid_resolution, uav_radius, safety_margin, obstacles);


% STEP 3: A* GLOBAL PATH PLANNING

[path, path_success, expanded_nodes, path_length] = ...
    astar(occupancy_grid, grid_x, grid_y, ...
    start_position, goal_position);

fprintf('\n============================================\n');
fprintf(' A* GLOBAL PATH INFORMATION\n');
fprintf('============================================\n');

fprintf('Path found          : %s\n', ...
    string(path_success));

fprintf('Expanded nodes      : %d\n', expanded_nodes);

if ~path_success
    error('A* could not find a path to the goal.');
end

fprintf('Path points         : %d\n', size(path, 1));
fprintf('Global path length  : %.3f m\n', path_length);

fprintf('============================================\n');


% STEP 4: LIDAR PARAMETERS

lidar_max_range = 5.0;
lidar_angle_resolution = 1.0;
lidar_orientation_deg = 0;


% STEP 5: UAV DYNAMICS PARAMETERS

time_step = 0.05;

maximum_time = 120;

maximum_steps = round(maximum_time / time_step);

maximum_acceleration = 2.0;
maximum_speed = 1.0;


% STEP 6: PD CONTROLLER PARAMETERS

Kp = 1.5;
Ki = 0;
Kd = 2.0;

integral_error = [0, 0];


% STEP 7: WAYPOINT PARAMETERS

waypoint_acceptance_radius = 0.10;

current_waypoint_index = 2;

reached_goal = false;
collision_detected = false;


% STEP 8: INITIALIZE UAV STATE

position = start_position;
velocity = [0, 0];

trajectory = zeros(maximum_steps + 1, 2);

trajectory(1, :) = position;

trajectory_count = 1;

simulation_steps = 0;


% STEP 9: ANIMATION PARAMETERS

visualization_interval = 10;

point_cloud_display_interval = 5;

figure('Name', 'Coupled UAV Navigation', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

hold on;
grid on;
axis equal;

xlim([0, room_width]);
ylim([0, room_height]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Coupled UAV - LiDAR - PD Navigation');


% DISPLAY OCCUPANCY GRID

imagesc(grid_x, grid_y, 1 - occupancy_grid);

set(gca, 'YDir', 'normal');

colormap(gray(2));
clim([0, 1]);


% DISPLAY GLOBAL PATH

h_path = plot(path(:,1), path(:,2), ...
    'c--', 'LineWidth', 1.5);


% DISPLAY START AND GOAL

h_start = plot(start_position(1), start_position(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

h_goal = plot(goal_position(1), goal_position(2), ...
    'r*', 'MarkerSize', 12, 'LineWidth', 2);


% INITIALIZE UAV AND TRAJECTORY

h_trajectory = plot(NaN, NaN, ...
    'b-', 'LineWidth', 2);

h_uav = plot(position(1), position(2), ...
    'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);


% INITIALIZE LIDAR BEAMS

h_lidar_beams = plot(NaN, NaN, ...
    'r-', 'LineWidth', 0.5);


% INITIALIZE POINT CLOUD

point_cloud = zeros(0, 2);

h_point_cloud = scatter(NaN, NaN, ...
    8, 'm', 'filled');


% STEP 10: COUPLED SIMULATION LOOP

fprintf('\nStarting coupled UAV navigation...\n');

for step = 1:maximum_steps

    simulation_steps = step;


    % ----------------------------------------------------------
    % 10.1: GENERATE LIDAR SCAN AT CURRENT UAV POSITION
    % ----------------------------------------------------------

    [lidar_ranges, lidar_angles, hit_points, hit_mask] = ...
        simulate_lidar(room_width, room_height, obstacles, ...
        position, lidar_orientation_deg, ...
        lidar_max_range, lidar_angle_resolution);


    % ----------------------------------------------------------
    % 10.2: UPDATE ACCUMULATED POINT CLOUD
    % ----------------------------------------------------------

    point_cloud = [point_cloud; hit_points(hit_mask, :)];

    
    % ----------------------------------------------------------
    % 10.3: SELECT CURRENT A* WAYPOINT
    % ----------------------------------------------------------

    target_position = path(current_waypoint_index, :);


    % ----------------------------------------------------------
    % 10.4: CALCULATE PD CONTROL COMMAND
    % ----------------------------------------------------------

    [acceleration_command, integral_error] = ...
        pid_controller(target_position, position, ...
        velocity, integral_error, time_step, ...
        Kp, Ki, Kd);


    % ----------------------------------------------------------
    % 10.5: UPDATE UAV DYNAMICS
    % ----------------------------------------------------------

    [new_position, new_velocity, actual_acceleration] = ...
        update_uav(position, velocity, acceleration_command, ...
        time_step, maximum_acceleration, maximum_speed);


    % ----------------------------------------------------------
    % 10.6: UPDATE UAV STATE
    % ----------------------------------------------------------

    position = new_position;
    velocity = new_velocity;

    trajectory_count = trajectory_count + 1;

    trajectory(trajectory_count, :) = position;


    % ----------------------------------------------------------
    % 10.7: CHECK DISCRETE COLLISION
    % ----------------------------------------------------------

    sample_column = floor(position(1) / grid_resolution) + 1;
    sample_row = floor(position(2) / grid_resolution) + 1;

    if sample_row < 1 || ...
       sample_row > size(occupancy_grid, 1) || ...
       sample_column < 1 || ...
       sample_column > size(occupancy_grid, 2)

        collision_detected = true;
        break;

    end

    if occupancy_grid(sample_row, sample_column) == 1

        collision_detected = true;
        break;

    end


    % ----------------------------------------------------------
    % 10.8: CHECK WAYPOINT REACHED
    % ----------------------------------------------------------

    distance_to_waypoint = norm(position - target_position);

    if distance_to_waypoint <= waypoint_acceptance_radius

        if current_waypoint_index < size(path, 1)

            current_waypoint_index = current_waypoint_index + 1;

        else

            if norm(velocity) <= 0.05

                reached_goal = true;
                break;

            end

        end

    end


    % ----------------------------------------------------------
    % 10.9: UPDATE ANIMATION AT SELECTED INTERVALS
    % ----------------------------------------------------------

    if mod(step, visualization_interval) == 0 || step == 1

        % UPDATE UAV POSITION

        set(h_uav, ...
            'XData', position(1), ...
            'YData', position(2));


        % UPDATE ACTUAL TRAJECTORY

        set(h_trajectory, ...
            'XData', trajectory(1:trajectory_count, 1), ...
            'YData', trajectory(1:trajectory_count, 2));


        % UPDATE LIDAR BEAMS
        % One graphics object displays all beams.

        number_of_beams = length(lidar_angles);

        beam_endpoints_x = position(1) + ...
            lidar_ranges .* cosd(lidar_angles);

        beam_endpoints_y = position(2) + ...
            lidar_ranges .* sind(lidar_angles);

        beam_x = [
            repmat(position(1), 1, number_of_beams);
            beam_endpoints_x;
            NaN(1, number_of_beams)
        ];

        beam_y = [
            repmat(position(2), 1, number_of_beams);
            beam_endpoints_y;
            NaN(1, number_of_beams)
        ];

        set(h_lidar_beams, ...
            'XData', beam_x(:), ...
            'YData', beam_y(:));


        % UPDATE DISPLAYED POINT CLOUD

        if mod(step, point_cloud_display_interval) == 0

            % Display a decimated copy to reduce rendering load.
            % The original point_cloud retains every detected point.

            displayed_point_cloud = point_cloud(1:5:end, :);

            set(h_point_cloud, ...
                'XData', displayed_point_cloud(:,1), ...
                'YData', displayed_point_cloud(:,2));

        end


        drawnow limitrate;

    end

end


% STEP 11: TRIM TRAJECTORY

trajectory = trajectory(1:trajectory_count, :);


% STEP 12: CONTINUOUS COLLISION VALIDATION

collision_check_resolution = 0.025;

continuous_collision_detected = false;

for segment_number = 1:size(trajectory, 1)-1

    segment_start = trajectory(segment_number, :);
    segment_end = trajectory(segment_number + 1, :);

    segment_length = norm(segment_end - segment_start);

    number_of_samples = max(1, ...
        ceil(segment_length / collision_check_resolution));

    for sample_number = 1:number_of_samples

        interpolation_factor = sample_number / number_of_samples;

        sample_position = segment_start + ...
            interpolation_factor * (segment_end - segment_start);

        sample_column = floor(sample_position(1) / grid_resolution) + 1;
        sample_row = floor(sample_position(2) / grid_resolution) + 1;

        if sample_row < 1 || ...
           sample_row > size(occupancy_grid, 1) || ...
           sample_column < 1 || ...
           sample_column > size(occupancy_grid, 2)

            continuous_collision_detected = true;
            break;

        end

        if occupancy_grid(sample_row, sample_column) == 1

            continuous_collision_detected = true;
            break;

        end

    end

    if continuous_collision_detected
        break;
    end

end


% STEP 13: CALCULATE PERFORMANCE METRICS

final_position_error = norm(position - goal_position);

final_speed = norm(velocity);

trajectory_length = sum(sqrt(sum(diff(trajectory).^2, 2)));

simulation_time = simulation_steps * time_step;

number_of_waypoints_reached = current_waypoint_index - 1;

if reached_goal
    number_of_waypoints_reached = size(path, 1);
end


% STEP 14: DISPLAY FINAL ANIMATION STATE

set(h_uav, ...
    'XData', position(1), ...
    'YData', position(2));

set(h_trajectory, ...
    'XData', trajectory(:,1), ...
    'YData', trajectory(:,2));

if ~isempty(point_cloud)

    displayed_point_cloud = point_cloud(1:5:end, :);

    set(h_point_cloud, ...
        'XData', displayed_point_cloud(:,1), ...
        'YData', displayed_point_cloud(:,2));

end

set(h_lidar_beams, 'XData', NaN, 'YData', NaN);

legend([h_path, h_trajectory, h_start, ...
    h_goal, h_uav, h_point_cloud], ...
    {'A* Global Path', ...
     'Actual UAV Trajectory', ...
     'Start Position', ...
     'Goal Position', ...
     'UAV', ...
     'Accumulated LiDAR Point Cloud'}, ...
     'Location', 'bestoutside');

drawnow;


% STEP 15: DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' COUPLED UAV NAVIGATION RESULTS\n');
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
    simulation_time);

fprintf('Simulation steps       : %d\n', ...
    simulation_steps);

fprintf('Global path length     : %.3f m\n', ...
    path_length);

fprintf('Actual trajectory length: %.3f m\n', ...
    trajectory_length);

fprintf('Waypoints reached      : %d / %d\n', ...
    number_of_waypoints_reached, size(path, 1));

fprintf('Goal reached           : %s\n', ...
    string(reached_goal));

fprintf('Discrete collision     : %s\n', ...
    string(collision_detected));

fprintf('Continuous collision   : %s\n', ...
    string(continuous_collision_detected));

fprintf('LiDAR points detected  : %d\n', ...
    size(point_cloud, 1));

fprintf('============================================\n');


% STEP 16: SAVE RESULTS IN WORKSPACE

% The following variables remain available after the script:
% trajectory
% point_cloud
% path
% occupancy_grid
% simulation_steps
% reached_goal
% collision_detected
% continuous_collision_detected
% final_position_error
% trajectory_length

hold off;
