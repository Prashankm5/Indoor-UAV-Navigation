
function plot_environment(room_width, room_height, ...
    obstacles, uav_radius, safety_margin, ...
    start_position, goal_position, ...
    occupancy_grid, grid_x, grid_y)

% ==============================================================
% PLOT ENVIRONMENT
% ==============================================================

figure('Name', 'Indoor Environment and UAV Safety Map', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

title('Indoor Environment with Inflated Obstacles and UAV Safety Clearance');

theta = linspace(0, 2*pi, 200); % radian

% ==============================================================
% PLOT 1 : PHYSICAL ENVIRONMENT
% ==============================================================

subplot(1, 2, 1);

hold on;
grid on;
axis equal;

xlim([0 room_width]);
ylim([0 room_height]);

rectangle('Position', [0 0 room_width room_height], ...
    'EdgeColor', 'k', 'LineWidth', 2);

for obstacle_number = 1:size(obstacles, 1)  % number of rows

    obstacle_x = obstacles(obstacle_number, 1); % row: obstacle_number, column: 1 
    obstacle_y = obstacles(obstacle_number, 2);
    obstacle_radius = obstacles(obstacle_number, 3);

    % Actual obstacle
    circle_x = obstacle_x + obstacle_radius*cos(theta);
    circle_y = obstacle_y + obstacle_radius*sin(theta);

    fill(circle_x, circle_y, [0.85 0.25 0.25], ...
        'EdgeColor', 'k');

    % Inflated safety boundary
    effective_radius = obstacle_radius + uav_radius + safety_margin;
    %disp(size(obstacle_radius));
    %disp(size(uav_radius));
    %disp(size(safety_margin));
    %size(effective_radius)
    %size(theta)
    inflated_x = obstacle_x + effective_radius*cos(theta);
    inflated_y = obstacle_y + effective_radius*sin(theta);

    plot(inflated_x, inflated_y, '--', ...
        'Color', [0.90 0.55 0.10], 'LineWidth', 1.5);

end

% Start and goal
plot(start_position(1), start_position(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(goal_position(1), goal_position(2), ...
    'b*', 'MarkerSize', 12, 'LineWidth', 1.5);

% Separate legend handles
h_room = plot(nan, nan, 'k-', 'LineWidth', 2);

h_obstacle = plot(nan, nan, 'o', ...
    'MarkerFaceColor', [0.85 0.25 0.25], ...
    'MarkerEdgeColor', 'k', 'MarkerSize', 7);

h_safety = plot(nan, nan, '--', ...
    'Color', [0.90 0.55 0.10], 'LineWidth', 1.5);

h_start = plot(nan, nan, 'go', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 9);

h_goal = plot(nan, nan, 'b*', ...
    'MarkerSize', 12, 'LineWidth', 1.5);

legend([h_room, h_obstacle, h_safety, h_start, h_goal], ...
    {'Room boundary', 'Actual obstacle', ...
    'Inflated safety boundary', 'UAV start', 'UAV goal'}, ...
    'Location', 'best');

title('Physical Environment');
xlabel('X Position (m)');
ylabel('Y Position (m)');

% ==============================================================
% PLOT 2 : OCCUPANCY GRID
% ==============================================================

subplot(1, 2, 2);

imagesc(grid_x, grid_y, occupancy_grid);

set(gca, 'YDir', 'normal');

axis equal;

xlim([0 room_width]);
ylim([0 room_height]);

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Occupancy Grid');

colormap(gca, [1 1 1; 0.15 0.15 0.15]);

colorbar('Ticks', [0.25 0.75], ...
    'TickLabels', {'Free', 'Occupied'});

hold on;

plot(start_position(1), start_position(2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(goal_position(1), goal_position(2), ...
    'b*', 'MarkerSize', 12, 'LineWidth', 1.5);

end
