
function [trajectory, point_cloud, metrics] = ...
    run_navigation_simulation(room_width, room_height, ...
    grid_resolution, occupancy_grid, grid_x, grid_y, ...
    obstacles, start_position, goal_position, path, path_length)

% ==============================================================
% RUN NAVIGATION SIMULATION
% APPROACH A : KNOWN MAP
%
% Runs:
%   1. LiDAR simulation
%   2. PD waypoint tracking
%   3. UAV dynamics
%   4. Point cloud accumulation
%   5. Collision validation
%   6. Live navigation animation
% ==============================================================


% ==============================================================
% STEP 1: SIMULATION PARAMETERS
% ==============================================================

lidar_max_range = 5.0;
lidar_angle_resolution = 1.0;
lidar_orientation_deg = 0;

time_step = 0.05;
maximum_time = 120;

maximum_steps = round(maximum_time / time_step);

maximum_acceleration = 2.0;
maximum_speed = 1.0;

Kp = 1.5;
Ki = 0;
Kd = 2.0;

waypoint_acceptance_radius = 0.10;

visualization_interval = 10;
point_cloud_display_interval = 5;
animation_delay = 0.001;

collision_check_resolution = 0.025;


% ==============================================================
% STEP 2: INITIALIZE UAV STATE
% ==============================================================

position = start_position;
velocity = [0, 0];

integral_error = [0, 0];

current_waypoint_index = 2;

reached_goal = false;
collision_detected = false;
continuous_collision_detected = false;

trajectory = zeros(maximum_steps + 1, 2);
trajectory(1, :) = position;

trajectory_count = 1;

point_cloud = zeros(0, 2);

number_of_lidar_scans = 0;
simulation_steps = 0;


% ==============================================================
% STEP 3: INITIALIZE ANIMATION
% ==============================================================

figure('Name', 'UAV Navigation - Approach A', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

imagesc(grid_x, grid_y, 1 - occupancy_grid);

set(gca, 'YDir', 'normal');

hold on;
grid on;
axis equal;

xlim([0, room_width]);
ylim([0, room_height]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Known-Map UAV Navigation with LiDAR');

colormap(gray(2));
clim([0, 1]);


% DISPLAY PHYSICAL OBSTACLES

theta = linspace(0, 2*pi, 100);

for obstacle_number = 1:size(obstacles, 1)

    obstacle_x = obstacles(obstacle_number, 1);
    obstacle_y = obstacles(obstacle_number, 2);
    obstacle_radius = obstacles(obstacle_number, 3);

    circle_x = obstacle_x + obstacle_radius*cos(theta);
    circle_y = obstacle_y + obstacle_radius*sin(theta);

    fill(circle_x, circle_y, [0.85, 0.25, 0.25], ...
        'EdgeColor', 'k', ...
        'HandleVisibility', 'off');

end


% DISPLAY PLANNED PATH

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

h_point_cloud = scatter(NaN, NaN, ...
    8, 'm', 'filled');


% INITIALIZE LIVE INFORMATION

h_status = text(0.2, room_height - 0.3, '', ...
    'FontSize', 9, ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'top', ...
    'BackgroundColor', 'w', ...
    'Margin', 5);


% ==============================================================
% STEP 4: COUPLED SIMULATION LOOP
% ==============================================================

fprintf('\nStarting UAV navigation simulation...\n');

for step = 1:maximum_steps

    simulation_steps = step;


    % ----------------------------------------------------------
    % 4.1: GENERATE LIDAR SCAN
    % ----------------------------------------------------------

    lidar_scan_position = position;

    [lidar_ranges, lidar_angles, hit_points, hit_mask] = ...
        simulate_lidar(room_width, room_height, obstacles, ...
        lidar_scan_position, lidar_orientation_deg, ...
        lidar_max_range, lidar_angle_resolution);

    number_of_lidar_scans = number_of_lidar_scans + 1;


    % ----------------------------------------------------------
    % 4.2: UPDATE COMPLETE POINT CLOUD
    % ----------------------------------------------------------

    new_points = hit_points(hit_mask, :);

    point_cloud = [point_cloud; new_points]; %#ok<AGROW>


    % ----------------------------------------------------------
    % 4.3: SELECT CURRENT WAYPOINT
    % ----------------------------------------------------------

    target_position = path(current_waypoint_index, :);


    % ----------------------------------------------------------
    % 4.4: PD CONTROLLER
    % ----------------------------------------------------------

    [acceleration_command, integral_error] = ...
        pid_controller(target_position, position, ...
        velocity, integral_error, time_step, ...
        Kp, Ki, Kd);


    % ----------------------------------------------------------
    % 4.5: UPDATE UAV DYNAMICS
    % ----------------------------------------------------------

    [new_position, new_velocity, actual_acceleration] = ...
        update_uav(position, velocity, acceleration_command, ...
        time_step, maximum_acceleration, maximum_speed);


    % ----------------------------------------------------------
    % 4.6: UPDATE UAV STATE
    % ----------------------------------------------------------

    position = new_position;
    velocity = new_velocity;

    trajectory_count = trajectory_count + 1;
    trajectory(trajectory_count, :) = position;


    % ----------------------------------------------------------
    % 4.7: DISCRETE COLLISION CHECK
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
    % 4.8: WAYPOINT REACHED CHECK
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
    % 4.9: UPDATE ANIMATION
    % ----------------------------------------------------------

    if mod(step, visualization_interval) == 0 || step == 1

        set(h_uav, ...
            'XData', position(1), ...
            'YData', position(2));


        set(h_trajectory, ...
            'XData', trajectory(1:trajectory_count, 1), ...
            'YData', trajectory(1:trajectory_count, 2));


        % UPDATE LIDAR BEAMS FROM SCAN POSITION

        number_of_beams = length(lidar_angles);

        beam_endpoints_x = lidar_scan_position(1) + ...
            lidar_ranges .* cosd(lidar_angles);

        beam_endpoints_y = lidar_scan_position(2) + ...
            lidar_ranges .* sind(lidar_angles);

        beam_x = [
            repmat(lidar_scan_position(1), 1, number_of_beams);
            beam_endpoints_x;
            NaN(1, number_of_beams)
        ];

        beam_y = [
            repmat(lidar_scan_position(2), 1, number_of_beams);
            beam_endpoints_y;
            NaN(1, number_of_beams)
        ];

        set(h_lidar_beams, ...
            'XData', beam_x(:), ...
            'YData', beam_y(:));


        % DISPLAY DECIMATED COPY ONLY

        if mod(step, point_cloud_display_interval) == 0

            displayed_point_cloud = point_cloud(1:5:end, :);

            set(h_point_cloud, ...
                'XData', displayed_point_cloud(:,1), ...
                'YData', displayed_point_cloud(:,2));

        end


        % LIVE INFORMATION

        distance_to_goal = norm(position - goal_position);

        status_text = sprintf([ ...
            'Time: %.2f s\n' ...
            'Position: [%.2f, %.2f] m\n' ...
            'Speed: %.2f m/s\n' ...
            'Waypoint: %d / %d\n' ...
            'Distance to goal: %.2f m\n' ...
            'LiDAR scans: %d\n' ...
            'Point measurements: %d'], ...
            step*time_step, ...
            position(1), position(2), ...
            norm(velocity), ...
            current_waypoint_index, size(path, 1), ...
            distance_to_goal, ...
            number_of_lidar_scans, ...
            size(point_cloud, 1));

        set(h_status, 'String', status_text);

        drawnow limitrate;

        pause(animation_delay);

    end

end


% ==============================================================
% STEP 5: TRIM TRAJECTORY
% ==============================================================

trajectory = trajectory(1:trajectory_count, :);


% ==============================================================
% STEP 6: CONTINUOUS COLLISION VALIDATION
% ==============================================================

for segment_number = 1:size(trajectory, 1)-1

    segment_start = trajectory(segment_number, :);
    segment_end = trajectory(segment_number + 1, :);

    segment_length = norm(segment_end - segment_start);

    number_of_samples = max(1, ...
        ceil(segment_length / collision_check_resolution));

    for sample_number = 0:number_of_samples

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


% ==============================================================
% STEP 7: CALCULATE PERFORMANCE METRICS
% ==============================================================

final_position_error = norm(position - goal_position);

final_speed = norm(velocity);

trajectory_length = sum(sqrt(sum(diff(trajectory).^2, 2)));

simulation_time = simulation_steps * time_step;

number_of_waypoints_reached = current_waypoint_index - 1;

if reached_goal
    number_of_waypoints_reached = size(path, 1);
end


if reached_goal

    simulation_status = 'GOAL REACHED';

elseif collision_detected || continuous_collision_detected

    simulation_status = 'COLLISION DETECTED';

else

    simulation_status = 'TIME LIMIT REACHED';

end


% ==============================================================
% STEP 8: UPDATE FINAL ANIMATION STATE
% ==============================================================

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

set(h_status, 'String', sprintf( ...
    'STATUS: %s\nFinal error: %.3f m\nFinal speed: %.3f m/s', ...
    simulation_status, final_position_error, final_speed));

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


% ==============================================================
% STEP 9: RETURN PERFORMANCE METRICS
% ==============================================================

metrics.initial_position = start_position;
metrics.goal_position = goal_position;
metrics.final_position = position;

metrics.final_position_error = final_position_error;
metrics.final_speed = final_speed;

metrics.simulation_time = simulation_time;
metrics.simulation_steps = simulation_steps;

metrics.global_path_length = path_length;
metrics.trajectory_length = trajectory_length;

metrics.total_waypoints = size(path, 1);
metrics.waypoints_reached = number_of_waypoints_reached;

metrics.number_of_lidar_scans = number_of_lidar_scans;
metrics.number_of_lidar_measurements = size(point_cloud, 1);

metrics.reached_goal = reached_goal;
metrics.collision_detected = collision_detected;
metrics.continuous_collision_detected = continuous_collision_detected;

metrics.simulation_status = simulation_status;


% ==============================================================
% STEP 10: DISPLAY RESULTS
% ==============================================================

fprintf('\n============================================\n');
fprintf(' COUPLED UAV NAVIGATION RESULTS\n');
fprintf('============================================\n');

fprintf('Initial position        : [%.3f, %.3f] m\n', ...
    start_position(1), start_position(2));

fprintf('Target position         : [%.3f, %.3f] m\n', ...
    goal_position(1), goal_position(2));

fprintf('Final position          : [%.3f, %.3f] m\n', ...
    position(1), position(2));

fprintf('Final position error    : %.5f m\n', final_position_error);

fprintf('Final speed             : %.5f m/s\n', final_speed);

fprintf('Simulation time         : %.2f s\n', simulation_time);

fprintf('Simulation steps        : %d\n', simulation_steps);

fprintf('Global path length      : %.3f m\n', path_length);

fprintf('Actual trajectory length: %.3f m\n', trajectory_length);

fprintf('Waypoints reached       : %d / %d\n', ...
    number_of_waypoints_reached, size(path, 1));

fprintf('LiDAR scans             : %d\n', number_of_lidar_scans);

fprintf('LiDAR measurements      : %d\n', size(point_cloud, 1));

fprintf('Goal reached            : %s\n', string(reached_goal));

fprintf('Discrete collision      : %s\n', string(collision_detected));

fprintf('Continuous collision    : %s\n', ...
    string(continuous_collision_detected));

fprintf('Simulation status       : %s\n', simulation_status);

fprintf('============================================\n');

end
