
function animate_navigation(room_width, room_height, ...
    occupancy_grid, grid_x, grid_y, path, trajectory, ...
    start_position, goal_position, obstacles, ...
    lidar_max_range, lidar_angle_resolution)

% ==============================================================
% ANIMATE NAVIGATION
% STAGE 8 : LIVE UAV + LIDAR + POINT CLOUD ANIMATION
% ==============================================================


% STEP 1: ANIMATION SETTINGS

trajectory_frame_interval = 10;
beam_animation_increment = 12;


% STEP 2: CREATE FIGURE

figure('Name', 'Indoor UAV Navigation', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

hold on;
grid on;
axis equal;

xlim([0, room_width]);
ylim([0, room_height]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Indoor UAV Navigation - Approach A');


% STEP 3: DISPLAY OCCUPANCY GRID

imagesc(grid_x, grid_y, 1 - occupancy_grid);

set(gca, 'YDir', 'normal');

colormap(gray(2));
clim([0, 1]);

uistack(gca, 'bottom');


% STEP 4: DISPLAY GLOBAL PATH

h_path = plot(path(:,1), path(:,2), ...
    'c--', 'LineWidth', 1.5);


% STEP 5: DISPLAY START AND GOAL

h_start = plot(start_position(1), start_position(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

h_goal = plot(goal_position(1), goal_position(2), ...
    'r*', 'MarkerSize', 12, 'LineWidth', 2);


% STEP 6: INITIALIZE UAV AND TRAJECTORY

h_trajectory = plot(NaN, NaN, ...
    'b-', 'LineWidth', 2);

h_uav = plot(trajectory(1,1), trajectory(1,2), ...
    'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);


% STEP 7: INITIALIZE LIDAR BEAM

h_lidar_beam = plot(NaN, NaN, ...
    'r-', 'LineWidth', 0.8);


% STEP 8: INITIALIZE POINT CLOUD

point_cloud = zeros(0, 2);

h_point_cloud = scatter(NaN, NaN, ...
    8, 'm', 'filled');


% STEP 9: INITIALIZE SCAN INFORMATION

number_of_trajectory_points = size(trajectory, 1);

lidar_orientation_deg = 0;

fprintf('\nStarting live navigation animation...\n');


% STEP 10: MOVE UAV AND GENERATE LIDAR SCANS

for trajectory_index = 1:trajectory_frame_interval: ...
        number_of_trajectory_points

    % CURRENT UAV POSITION

    current_position = trajectory(trajectory_index, :);


    % GENERATE A NEW LIDAR SCAN

    [lidar_ranges, lidar_angles, hit_points, hit_mask] = ...
        simulate_lidar(room_width, room_height, obstacles, ...
        current_position, lidar_orientation_deg, ...
        lidar_max_range, lidar_angle_resolution);


    % ACCUMULATE ALL DETECTED LIDAR POINTS

    point_cloud = [point_cloud; hit_points(hit_mask, :)];

    set(h_point_cloud, ...
        'XData', point_cloud(:,1), ...
        'YData', point_cloud(:,2));


    % UPDATE UAV TRAJECTORY

    set(h_trajectory, ...
        'XData', trajectory(1:trajectory_index, 1), ...
        'YData', trajectory(1:trajectory_index, 2));

    set(h_uav, ...
        'XData', current_position(1), ...
        'YData', current_position(2));


    % ANIMATE INDIVIDUAL LIDAR BEAMS

    for beam_index = 1:beam_animation_increment: ...
            length(lidar_angles)

        % CURRENT BEAM ANGLE

        current_angle = lidar_angles(beam_index);


        % CALCULATE BEAM ENDPOINT

        beam_endpoint = current_position + ...
            lidar_ranges(beam_index) * ...
            [cosd(current_angle), sind(current_angle)];


        % DISPLAY CURRENT BEAM

        set(h_lidar_beam, ...
            'XData', [current_position(1), beam_endpoint(1)], ...
            'YData', [current_position(2), beam_endpoint(2)]);


        drawnow limitrate;
        pause(0.001);

    end

end


% STEP 11: DISPLAY FINAL UAV POSITION

set(h_lidar_beam, 'XData', NaN, 'YData', NaN);

set(h_uav, ...
    'XData', trajectory(end,1), ...
    'YData', trajectory(end,2));

set(h_trajectory, ...
    'XData', trajectory(:,1), ...
    'YData', trajectory(:,2));


% STEP 12: DISPLAY LEGEND

legend([h_path, h_trajectory, h_start, ...
    h_goal, h_uav, h_point_cloud], ...
    {'A* Global Path', ...
     'Actual UAV Trajectory', ...
     'Start Position', ...
     'Goal Position', ...
     'UAV', ...
     'Accumulated LiDAR Point Cloud'}, ...
     'Location', 'bestoutside');


% STEP 13: DISPLAY FINAL RESULTS

fprintf('\n============================================\n');
fprintf(' LIVE NAVIGATION ANIMATION COMPLETED\n');
fprintf('============================================\n');

fprintf('Trajectory positions : %d\n', ...
    number_of_trajectory_points);

fprintf('LiDAR points detected: %d\n', ...
    size(point_cloud, 1));

fprintf('============================================\n');

hold off;

end
