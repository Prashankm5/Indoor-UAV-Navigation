
% ==============================================================
% TEST : SCENARIO VALIDATION
% APPROACH A : MULTIPLE-ENVIRONMENT VALIDATION
% ==============================================================

clear;
clc;
close all;


% STEP 1: ADD PROJECT FOLDERS TO MATLAB PATH

project_folder = fileparts(fileparts(mfilename('fullpath')));

addpath(genpath(project_folder));


% STEP 2: COMMON ENVIRONMENT PARAMETERS

room_width = 10;
room_height = 10;

grid_resolution = 0.1;

uav_radius = 0.25;
safety_margin = 0.15;


% STEP 3: LOAD TEST SCENARIOS

[scenario_names, scenario_obstacles, ...
    start_positions, goal_positions] = ...
    create_test_scenarios();


number_of_scenarios = length(scenario_names);


% STEP 4: INITIALIZE RESULTS

scenario_validation = cell(number_of_scenarios, 5);


% STEP 5: VALIDATE EACH SCENARIO

for scenario_number = 1:number_of_scenarios

    fprintf('\n============================================\n');

    fprintf('VALIDATING: %s\n', ...
        scenario_names{scenario_number});

    fprintf('============================================\n');


    % GET CURRENT SCENARIO

    obstacles = scenario_obstacles{scenario_number};

    start_position = start_positions(scenario_number, :);

    goal_position = goal_positions(scenario_number, :);


    % CREATE OCCUPANCY GRID

    [occupancy_grid, grid_x, grid_y] = ...
        create_occupancy_grid(room_width, room_height, ...
        grid_resolution, uav_radius, safety_margin, obstacles);


    % RUN A* PATH PLANNING

    [path, path_success, expanded_nodes, path_length] = ...
        astar(occupancy_grid, grid_x, grid_y, ...
        start_position, goal_position);


    % STORE RESULTS

    scenario_validation{scenario_number, 1} = ...
        scenario_names{scenario_number};

    scenario_validation{scenario_number, 2} = ...
        size(obstacles, 1);

    scenario_validation{scenario_number, 3} = ...
        path_success;

    scenario_validation{scenario_number, 4} = ...
        expanded_nodes;

    scenario_validation{scenario_number, 5} = ...
        path_length;


    % DISPLAY RESULTS

    fprintf('Number of obstacles : %d\n', size(obstacles, 1));

    fprintf('Start position      : [%.2f, %.2f]\n', ...
        start_position(1), start_position(2));

    fprintf('Goal position       : [%.2f, %.2f]\n', ...
        goal_position(1), goal_position(2));

    fprintf('Path found          : %s\n', string(path_success));

    fprintf('Expanded nodes      : %d\n', expanded_nodes);

    if path_success
        fprintf('Path length         : %.3f m\n', path_length);
    else
        fprintf('Path length         : Not available\n');
    end

end


% STEP 6: DISPLAY COMBINED VALIDATION RESULTS

validation_table = cell2table(scenario_validation, ...
    'VariableNames', { ...
    'Scenario', ...
    'Number_of_Obstacles', ...
    'Astar_Success', ...
    'Expanded_Nodes', ...
    'Path_Length_m'});


fprintf('\n============================================\n');
fprintf(' COMBINED SCENARIO VALIDATION\n');
fprintf('============================================\n');

disp(validation_table);


% STEP 7: FINAL VALIDATION STATUS

if all(cell2mat(scenario_validation(:, 3)))

    fprintf('\nAll scenarios have a feasible A* path.\n');

else

    fprintf('\nWARNING: One or more scenarios failed A* validation.\n');

end

fprintf('============================================\n');





% ==============================================================
% STEP 8: VISUALIZE ALL FIVE TEST ENVIRONMENTS
% ==============================================================

figure('Name', 'Approach A - Scenario Validation', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

for scenario_number = 1:number_of_scenarios

    % CURRENT SCENARIO

    obstacles = scenario_obstacles{scenario_number};

    start_position = start_positions(scenario_number, :);

    goal_position = goal_positions(scenario_number, :);


    % CREATE OCCUPANCY GRID

    [occupancy_grid, grid_x, grid_y] = ...
        create_occupancy_grid(room_width, room_height, ...
        grid_resolution, uav_radius, safety_margin, obstacles);


    % CALCULATE A* PATH

    [path, path_success, ~, path_length] = ...
        astar(occupancy_grid, grid_x, grid_y, ...
        start_position, goal_position);


    % CREATE SUBPLOT

    subplot(2, 3, scenario_number);

    imagesc(grid_x, grid_y, occupancy_grid);

    set(gca, 'YDir', 'normal');

    axis equal;

    xlim([0 room_width]);
    ylim([0 room_height]);

    colormap(gca, [1 1 1; 0.15 0.15 0.15]);

    hold on;
    grid on;


    % DRAW ACTUAL OBSTACLES

    theta = linspace(0, 2*pi, 100);

    for obstacle_number = 1:size(obstacles, 1)

        obstacle_x = obstacles(obstacle_number, 1);
        obstacle_y = obstacles(obstacle_number, 2);
        obstacle_radius = obstacles(obstacle_number, 3);

        circle_x = obstacle_x + obstacle_radius*cos(theta);
        circle_y = obstacle_y + obstacle_radius*sin(theta);

        plot(circle_x, circle_y, 'r-', 'LineWidth', 1.2);

    end


    % DRAW A* PATH

    if path_success

        plot(path(:,1), path(:,2), ...
            'b-', 'LineWidth', 2);

    end


    % DRAW START AND GOAL

    plot(start_position(1), start_position(2), ...
        'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);

    plot(goal_position(1), goal_position(2), ...
        'r*', 'MarkerSize', 10, 'LineWidth', 1.5);


    % TITLE

    if path_success

        title(sprintf('%s\nPath length: %.3f m', ...
            scenario_names{scenario_number}, path_length), ...
            'Interpreter', 'none');

    else

        title(sprintf('%s\nNO PATH', ...
            scenario_names{scenario_number}), ...
            'Interpreter', 'none');

    end


    xlabel('X Position (m)');
    ylabel('Y Position (m)');

    hold off;

end


sgtitle('Approach A: Five Test Environments and A* Paths');

