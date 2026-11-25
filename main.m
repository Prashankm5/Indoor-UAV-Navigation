
% ==============================================================
% MAIN : INDOOR UAV NAVIGATION
% APPROACH A : KNOWN MAP
% MULTI-SCENARIO VALIDATION
%
% Stages:
%   1. Environment generation
%   2. Initial LiDAR simulation and animation
%   3. LiDAR point-cloud processing
%   4. A* global path planning
%   5. Coupled UAV + LiDAR + PD navigation
%   6. Performance evaluation and data saving
%
% Results are stored directly inside the Results folder.
% Each scenario has only one folder containing its latest results.
%
% Run this file to execute the selected scenarios.
% ==============================================================

clear;
clc;
close all;


% ==============================================================
% STEP 1: PROJECT PATH
% ==============================================================

project_folder = fileparts(mfilename('fullpath'));

results_root = fullfile(project_folder, 'Results');

% Add project folders to MATLAB path.

addpath(genpath(project_folder));

% Remove Results and its subfolders from MATLAB path.

if exist(results_root, 'dir')

    rmpath(genpath(results_root));

end


% ==============================================================
% STEP 2: SELECT SCENARIOS
% ==============================================================

% Available scenarios:
%
% 1 = E1_Reference
% 2 = E2_Relocated_Obstacles
% 3 = E3_Narrow_Passage
% 4 = E4_Cluttered_Environment
% 5 = E5_Different_Start_Goal

%selected_scenarios = [1, 3, 5];

% To run all scenarios:
selected_scenarios = 1:5;


% ==============================================================
% STEP 3: COMMON ENVIRONMENT PARAMETERS
% ==============================================================

room_width = 10;
room_height = 10;

grid_resolution = 0.1;

uav_radius = 0.25;
safety_margin = 0.15;


% ==============================================================
% STEP 4: COMMON LIDAR PARAMETERS
% ==============================================================

lidar_max_range = 5.0;

lidar_angle_resolution = 1.0;

lidar_orientation_deg = 0;

animation_delay = 0.01;


% ==============================================================
% STEP 5: LOAD TEST SCENARIOS
% ==============================================================

[scenario_names, scenario_obstacles, ...
    start_positions, goal_positions] = create_test_scenarios();


number_of_scenarios = length(scenario_names);


if isempty(selected_scenarios)

    error('No scenarios have been selected.');

end


if any(selected_scenarios < 1) || ...
   any(selected_scenarios > number_of_scenarios) || ...
   any(mod(selected_scenarios, 1) ~= 0)

    error('Invalid scenario selection.');

end


% ==============================================================
% STEP 6: CREATE RESULTS FOLDER
% ==============================================================

results_root = fullfile(project_folder, 'Results');

if ~exist(results_root, 'dir')

    mkdir(results_root);

end


fprintf('\n==================================================\n');
fprintf(' INDOOR UAV NAVIGATION - APPROACH A\n');
fprintf(' MULTI-SCENARIO VALIDATION\n');
fprintf('==================================================\n');

fprintf('Selected scenarios : %d\n', length(selected_scenarios));

fprintf('Results folder     : %s\n', results_root);

fprintf('==================================================\n');


% ==============================================================
% STEP 7: INITIALIZE CURRENT RUN SUMMARY TABLE
% ==============================================================

summary_table = table('Size', [0, 16], ...
    'VariableTypes', { ...
    'string', 'logical', 'logical', ...
    'double', 'double', 'double', 'double', ...
    'double', 'double', 'double', 'double', ...
    'double', 'logical', 'logical', 'string', 'string'}, ...
    'VariableNames', { ...
    'Scenario', ...
    'Path_Found', ...
    'Goal_Reached', ...
    'Expanded_Nodes', ...
    'Global_Path_Length_m', ...
    'Trajectory_Length_m', ...
    'Final_Position_Error_m', ...
    'Final_Speed_mps', ...
    'Simulation_Time_s', ...
    'Simulation_Steps', ...
    'LiDAR_Scans', ...
    'LiDAR_Measurements', ...
    'Collision_Detected', ...
    'Continuous_Collision_Detected', ...
    'Simulation_Status', ...
    'Error_Message'});


% ==============================================================
% STEP 8: EXECUTE SELECTED SCENARIOS
% ==============================================================

for selected_number = 1:length(selected_scenarios)

    scenario_number = selected_scenarios(selected_number);

    scenario_name = char(scenario_names{scenario_number});

    obstacles = scenario_obstacles{scenario_number};

    start_position = start_positions(scenario_number, :);

    goal_position = goal_positions(scenario_number, :);


    % ----------------------------------------------------------
    % CREATE SCENARIO RESULTS FOLDER
    % ----------------------------------------------------------

    scenario_folder = fullfile(results_root, scenario_name);


    % Remove only the selected scenario's previous results.
    % Other scenario folders remain untouched.

    if exist(scenario_folder, 'dir')

        rmdir(scenario_folder, 's');

    end

    mkdir(scenario_folder);


    % Keep track of figures created for this scenario only.
    % Figures from previous scenarios remain open.

    scenario_figure_handles = {};


    fprintf('\n\n==================================================\n');
    fprintf(' SCENARIO %d: %s\n', scenario_number, scenario_name);
    fprintf('==================================================\n');

    fprintf('Start position : [%.2f, %.2f] m\n', ...
        start_position(1), start_position(2));

    fprintf('Goal position  : [%.2f, %.2f] m\n', ...
        goal_position(1), goal_position(2));

    fprintf('Obstacles      : %d\n', size(obstacles, 1));

    fprintf('Results folder : %s\n', scenario_folder);

    fprintf('==================================================\n');


    % ==========================================================
    % STEP 8.1: CREATE OCCUPANCY GRID
    % ==========================================================

    try

        [occupancy_grid, grid_x, grid_y] = ...
            create_occupancy_grid(room_width, room_height, ...
            grid_resolution, uav_radius, safety_margin, obstacles);


        fprintf('\n============================================\n');
        fprintf(' ENVIRONMENT INFORMATION\n');
        fprintf('============================================\n');

        fprintf('Room dimensions     : %.2f x %.2f m\n', ...
            room_width, room_height);

        fprintf('Grid resolution     : %.2f m\n', grid_resolution);

        fprintf('Grid size           : %d x %d\n', ...
            size(occupancy_grid, 1), size(occupancy_grid, 2));

        fprintf('Number of obstacles : %d\n', size(obstacles, 1));

        fprintf('Occupied cells      : %d\n', sum(occupancy_grid(:)));

        fprintf('Free cells          : %d\n', sum(~occupancy_grid(:)));

        fprintf('============================================\n');


        % ======================================================
        % STEP 8.2: PLOT ENVIRONMENT
        % ======================================================

        plot_environment(room_width, room_height, ...
            obstacles, uav_radius, safety_margin, ...
            start_position, goal_position, ...
            occupancy_grid, grid_x, grid_y);

        environment_figure = gcf;

        set(environment_figure, 'Name', ...
            [scenario_name ' - Environment']);

        scenario_figure_handles{end+1} = environment_figure;


        % ======================================================
        % STEP 8.3: INITIAL LIDAR SIMULATION
        % ======================================================

        lidar_position = start_position;


        [lidar_ranges, lidar_angles, hit_points, hit_mask] = ...
            simulate_lidar(room_width, room_height, obstacles, ...
            lidar_position, lidar_orientation_deg, ...
            lidar_max_range, lidar_angle_resolution);


        fprintf('\n============================================\n');
        fprintf(' INITIAL LIDAR SIMULATION\n');
        fprintf('============================================\n');

        fprintf('LiDAR position       : [%.2f, %.2f] m\n', ...
            lidar_position(1), lidar_position(2));

        fprintf('Maximum range        : %.2f m\n', lidar_max_range);

        fprintf('Angular resolution   : %.2f degrees\n', ...
            lidar_angle_resolution);

        fprintf('Total beams          : %d\n', length(lidar_angles));

        fprintf('Detected beams       : %d\n', sum(hit_mask));

        fprintf('No-hit beams         : %d\n', sum(~hit_mask));

        fprintf('============================================\n');


        % ======================================================
        % STEP 8.4: ANIMATE INITIAL LIDAR SCAN
        % ======================================================

        animate_lidar(room_width, room_height, obstacles, ...
            lidar_position, lidar_angles, hit_points, hit_mask, ...
            animation_delay);

        lidar_animation_figure = gcf;

        set(lidar_animation_figure, 'Name', ...
            [scenario_name ' - Initial LiDAR Scan']);

        scenario_figure_handles{end+1} = lidar_animation_figure;


        % ======================================================
        % STEP 8.5: PROCESS INITIAL LIDAR SCAN
        % ======================================================

        [detected_points, front_distance, left_distance, ...
            rear_distance, right_distance] = ...
            process_lidar_scan(lidar_ranges, lidar_angles, ...
            hit_points, hit_mask, lidar_max_range);


        fprintf('\n============================================\n');
        fprintf(' LIDAR SCAN PROCESSING\n');
        fprintf('============================================\n');

        fprintf('Detected surface points : %d\n', ...
            size(detected_points, 1));

        fprintf('Front minimum distance  : %.3f m\n', front_distance);

        fprintf('Left minimum distance   : %.3f m\n', left_distance);

        fprintf('Rear minimum distance   : %.3f m\n', rear_distance);

        fprintf('Right minimum distance  : %.3f m\n', right_distance);

        fprintf('============================================\n');


        % ======================================================
        % STEP 8.6: OBSTACLE PROXIMITY DETECTION
        % ======================================================

        warning_distance = 2.0;

        [front_status, left_status, rear_status, right_status] = ...
            detect_obstacles(front_distance, left_distance, ...
            rear_distance, right_distance, ...
            uav_radius, safety_margin, warning_distance);


        fprintf('\n============================================\n');
        fprintf(' OBSTACLE PROXIMITY DETECTION\n');
        fprintf('============================================\n');

        fprintf('Required safety distance : %.2f m\n', ...
            uav_radius + safety_margin);

        fprintf('Warning distance         : %.2f m\n', ...
            warning_distance);

        fprintf('Front : %-10s (%.3f m)\n', ...
            front_status, front_distance);

        fprintf('Left  : %-10s (%.3f m)\n', ...
            left_status, left_distance);

        fprintf('Rear  : %-10s (%.3f m)\n', ...
            rear_status, rear_distance);

        fprintf('Right : %-10s (%.3f m)\n', ...
            right_status, right_distance);

        fprintf('============================================\n');


        % ======================================================
        % STEP 8.7: PLOT INITIAL LIDAR POINT CLOUD
        % ======================================================

        point_cloud_figure = figure( ...
            'Name', [scenario_name ' - Initial LiDAR Point Cloud'], ...
            'NumberTitle', 'off', ...
            'Color', 'w');

        scenario_figure_handles{end+1} = point_cloud_figure;

        hold on;
        grid on;
        axis equal;

        xlim([0, room_width]);
        ylim([0, room_height]);

        xlabel('X Position (m)');
        ylabel('Y Position (m)');

        rectangle('Position', [0, 0, room_width, room_height], ...
            'EdgeColor', 'k', ...
            'LineWidth', 2);


        theta = linspace(0, 2*pi, 200);

        for obstacle_number = 1:size(obstacles, 1)

            obstacle_x = obstacles(obstacle_number, 1);
            obstacle_y = obstacles(obstacle_number, 2);
            obstacle_radius = obstacles(obstacle_number, 3);

            circle_x = obstacle_x + obstacle_radius*cos(theta);
            circle_y = obstacle_y + obstacle_radius*sin(theta);

            fill(circle_x, circle_y, [0.85, 0.25, 0.25], ...
                'EdgeColor', 'k');

        end


        if ~isempty(detected_points)

            scatter(detected_points(:,1), detected_points(:,2), ...
                12, [0.00, 0.65, 0.15], 'filled');

        end


        plot(lidar_position(1), lidar_position(2), ...
            'bo', ...
            'MarkerFaceColor', 'b', ...
            'MarkerSize', 9);

        title('Initial LiDAR Point Cloud');

        legend('Room boundary', 'Actual obstacles', ...
            'Detected LiDAR points', 'LiDAR position', ...
            'Location', 'best');

        hold off;


        % ======================================================
        % STEP 8.8: A* GLOBAL PATH PLANNING
        % ======================================================

        [path, path_success, expanded_nodes, path_length] = ...
            astar(occupancy_grid, grid_x, grid_y, ...
            start_position, goal_position);


        fprintf('\n============================================\n');
        fprintf(' A* GLOBAL PATH INFORMATION\n');
        fprintf('============================================\n');

        fprintf('Path found          : %s\n', string(path_success));

        fprintf('Expanded nodes      : %d\n', expanded_nodes);


        if ~path_success

            fprintf('A* could not find a path for this scenario.\n');


            failure_row = table( ...
                string(scenario_name), ...
                false, false, ...
                expanded_nodes, ...
                NaN, NaN, NaN, NaN, ...
                NaN, NaN, NaN, NaN, ...
                false, false, ...
                "A* PATH NOT FOUND", ...
                "No collision-free global path found.", ...
                'VariableNames', summary_table.Properties.VariableNames);


            summary_table = [summary_table; failure_row];


            % Save environment data even when A* fails.

            writematrix(obstacles, ...
                fullfile(scenario_folder, 'obstacles.csv'));

            writematrix(occupancy_grid, ...
                fullfile(scenario_folder, 'occupancy_grid.csv'));


            % Save figures generated before the A* failure.

            save_scenario_figures(scenario_figure_handles, scenario_folder);


            fprintf('\nScenario skipped. Moving to next scenario.\n');

            continue;

        end


        fprintf('Path points         : %d\n', size(path, 1));

        fprintf('Global path length  : %.3f m\n', path_length);

        fprintf('============================================\n');


        % ======================================================
        % STEP 8.9: PLOT A* GLOBAL PATH
        % ======================================================

        path_figure = figure( ...
            'Name', [scenario_name ' - A* Global Path'], ...
            'NumberTitle', 'off', ...
            'Color', 'w');

        scenario_figure_handles{end+1} = path_figure;

        imagesc(grid_x, grid_y, occupancy_grid);

        set(gca, 'YDir', 'normal');

        axis equal;

        xlim([0, room_width]);
        ylim([0, room_height]);

        colormap(gca, [1, 1, 1; 0.15, 0.15, 0.15]);

        hold on;
        grid on;

        h_path = plot(path(:,1), path(:,2), ...
            'b-', 'LineWidth', 2);

        h_start = plot(start_position(1), start_position(2), ...
            'go', ...
            'MarkerFaceColor', 'g', ...
            'MarkerSize', 9);

        h_goal = plot(goal_position(1), goal_position(2), ...
            'r*', ...
            'MarkerSize', 12, ...
            'LineWidth', 1.5);

        legend([h_path, h_start, h_goal], ...
            {'A* planned path', 'Start', 'Goal'}, ...
            'Location', 'best');

        title('A* Global Path Planning');

        xlabel('X Position (m)');
        ylabel('Y Position (m)');

        hold off;


        % ======================================================
        % STEP 8.10: SAVE INPUT DATA
        % ======================================================

        writematrix(obstacles, ...
            fullfile(scenario_folder, 'obstacles.csv'));

        writematrix(occupancy_grid, ...
            fullfile(scenario_folder, 'occupancy_grid.csv'));

        writematrix(path, ...
            fullfile(scenario_folder, 'planned_path.csv'));

        writematrix(detected_points, ...
            fullfile(scenario_folder, 'initial_lidar_point_cloud.csv'));


        parameters_table = table( ...
            room_width, room_height, grid_resolution, ...
            uav_radius, safety_margin, ...
            lidar_max_range, lidar_angle_resolution, ...
            lidar_orientation_deg, ...
            start_position(1), start_position(2), ...
            goal_position(1), goal_position(2), ...
            'VariableNames', { ...
            'Room_Width_m', 'Room_Height_m', 'Grid_Resolution_m', ...
            'UAV_Radius_m', 'Safety_Margin_m', ...
            'LiDAR_Max_Range_m', 'LiDAR_Angular_Resolution_deg', ...
            'LiDAR_Orientation_deg', ...
            'Start_X_m', 'Start_Y_m', 'Goal_X_m', 'Goal_Y_m'});


        writetable(parameters_table, ...
            fullfile(scenario_folder, 'parameters.csv'));


        % ======================================================
        % STEP 8.11: RUN COUPLED UAV NAVIGATION SIMULATION
        % ======================================================

        fprintf('\nStarting coupled simulation for %s...\n', ...
            scenario_name);


        [trajectory, point_cloud, metrics] = ...
            run_navigation_simulation(room_width, room_height, ...
            grid_resolution, occupancy_grid, grid_x, grid_y, ...
            obstacles, start_position, goal_position, ...
            path, path_length);


        % Capture the coupled animation figure created by the function.

        coupled_figure = gcf;

        set(coupled_figure, 'Name', ...
            [scenario_name ' - Coupled UAV Navigation']);

        scenario_figure_handles{end+1} = coupled_figure;


        % ======================================================
        % STEP 8.12: SAVE SIMULATION OUTPUTS
        % ==============================================================

        writematrix(trajectory, ...
            fullfile(scenario_folder, 'trajectory.csv'));

        writematrix(point_cloud, ...
            fullfile(scenario_folder, 'lidar_point_cloud.csv'));


        % Save scalar metrics and split position vectors into
        % individual X and Y columns.

        metrics_table = table( ...
            metrics.initial_position(1), ...
            metrics.initial_position(2), ...
            metrics.goal_position(1), ...
            metrics.goal_position(2), ...
            metrics.final_position(1), ...
            metrics.final_position(2), ...
            metrics.final_position_error, ...
            metrics.final_speed, ...
            metrics.simulation_time, ...
            metrics.simulation_steps, ...
            metrics.global_path_length, ...
            metrics.trajectory_length, ...
            metrics.total_waypoints, ...
            metrics.waypoints_reached, ...
            metrics.number_of_lidar_scans, ...
            metrics.number_of_lidar_measurements, ...
            metrics.reached_goal, ...
            metrics.collision_detected, ...
            metrics.continuous_collision_detected, ...
            string(metrics.simulation_status), ...
            'VariableNames', { ...
            'Initial_X_m', 'Initial_Y_m', ...
            'Goal_X_m', 'Goal_Y_m', ...
            'Final_X_m', 'Final_Y_m', ...
            'Final_Position_Error_m', ...
            'Final_Speed_mps', ...
            'Simulation_Time_s', ...
            'Simulation_Steps', ...
            'Global_Path_Length_m', ...
            'Trajectory_Length_m', ...
            'Total_Waypoints', ...
            'Waypoints_Reached', ...
            'LiDAR_Scans', ...
            'LiDAR_Measurements', ...
            'Goal_Reached', ...
            'Collision_Detected', ...
            'Continuous_Collision_Detected', ...
            'Simulation_Status'});


        writetable(metrics_table, ...
            fullfile(scenario_folder, 'performance_metrics.csv'));


        % ======================================================
        % STEP 8.13: ADD SCENARIO TO CURRENT RUN SUMMARY
        % ======================================================

        summary_row = table( ...
            string(scenario_name), ...
            true, ...
            metrics.reached_goal, ...
            expanded_nodes, ...
            metrics.global_path_length, ...
            metrics.trajectory_length, ...
            metrics.final_position_error, ...
            metrics.final_speed, ...
            metrics.simulation_time, ...
            metrics.simulation_steps, ...
            metrics.number_of_lidar_scans, ...
            metrics.number_of_lidar_measurements, ...
            metrics.collision_detected, ...
            metrics.continuous_collision_detected, ...
            string(metrics.simulation_status), ...
            "", ...
            'VariableNames', summary_table.Properties.VariableNames);


        summary_table = [summary_table; summary_row];


        % ======================================================
        % STEP 8.14: SAVE THIS SCENARIO'S FIGURES
        % ======================================================

        save_scenario_figures(scenario_figure_handles, scenario_folder);


        % ======================================================
        % STEP 8.15: SCENARIO COMPLETION INFORMATION
        % ======================================================

        fprintf('\n==================================================\n');
        fprintf(' SCENARIO COMPLETED: %s\n', scenario_name);
        fprintf('==================================================\n');

        fprintf('Goal reached          : %s\n', ...
            string(metrics.reached_goal));

        fprintf('Final position error  : %.5f m\n', ...
            metrics.final_position_error);

        fprintf('Collision detected    : %s\n', ...
            string(metrics.collision_detected));

        fprintf('Simulation status     : %s\n', ...
            string(metrics.simulation_status));

        fprintf('Scenario results      : %s\n', scenario_folder);

        fprintf('==================================================\n');


    catch ME

        % ======================================================
        % STEP 8.16: HANDLE SCENARIO FAILURE
        % ==============================================================

        fprintf('\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');

        fprintf('SCENARIO FAILED: %s\n', scenario_name);

        fprintf('Error message: %s\n', ME.message);

        fprintf('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');


        failure_row = table( ...
            string(scenario_name), ...
            false, false, ...
            NaN, NaN, NaN, NaN, NaN, ...
            NaN, NaN, NaN, NaN, ...
            false, false, ...
            "EXECUTION FAILED", ...
            string(ME.message), ...
            'VariableNames', summary_table.Properties.VariableNames);


        summary_table = [summary_table; failure_row];


        % Save any figures successfully created before the error.

        try

            save_scenario_figures(scenario_figure_handles, scenario_folder);

        catch

            fprintf('Could not save all figures for this scenario.\n');

        end


        fprintf('Moving to the next selected scenario...\n');

    end

end


% ==============================================================
% STEP 9: UPDATE PERMANENT COMBINED SUMMARY
% ==============================================================

summary_filepath = fullfile(results_root, 'combined_summary.csv');


% Load the previous summary if it exists.

if exist(summary_filepath, 'file')

    previous_summary = readtable(summary_filepath, ...
        'TextType', 'string');


    % Identify the scenarios executed in the current run.

    selected_scenario_names = string( ...
        scenario_names(selected_scenarios));


    % Remove old summary rows only for scenarios executed now.
    % Results of unselected scenarios are retained.

    previous_summary( ...
        ismember(previous_summary.Scenario, selected_scenario_names), :) = [];


    % Combine previous results with the latest results.

    summary_table = [previous_summary; summary_table];

end


% Save the updated permanent summary.

writetable(summary_table, summary_filepath);


% ==============================================================
% STEP 10: FINAL EXPERIMENT SUMMARY
% ==============================================================

fprintf('\n\n==================================================\n');
fprintf(' MULTI-SCENARIO EXPERIMENT COMPLETED\n');
fprintf('==================================================\n');

fprintf('Scenarios selected        : %d\n', ...
    length(selected_scenarios));

fprintf('Scenarios in summary      : %d\n', ...
    height(summary_table));

fprintf('Goal reached               : %d\n', ...
    sum(summary_table.Goal_Reached));

fprintf('Combined summary           : %s\n', ...
    summary_filepath);

fprintf('Results folder             : %s\n', ...
    results_root);

fprintf('==================================================\n');


disp(summary_table);


% ==============================================================
% LOCAL FUNCTION: SAVE SCENARIO FIGURES
% ==============================================================

function save_scenario_figures(figure_handles, scenario_folder)

    for figure_number = 1:length(figure_handles)

        current_figure = figure_handles{figure_number};

        if isempty(current_figure) || ~isgraphics(current_figure)

            continue;

        end


        figure_filename = sprintf( ...
            'Figure_%02d.png', figure_number);

        figure_filepath = fullfile( ...
            scenario_folder, figure_filename);


        exportgraphics(current_figure, figure_filepath, ...
            'Resolution', 300);

    end

end
