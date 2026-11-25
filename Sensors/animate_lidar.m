
function animate_lidar(room_width, room_height, obstacles, ...
    lidar_position, lidar_angles, hit_points, hit_mask, ...
    animation_delay)

% ==============================================================
% ANIMATE LIDAR SCAN
% ==============================================================

%figure('Name', 'Animated LiDAR Scan', 'Color', 'w');
figure('Name', 'Initial 360 Degree LiDAR Scan', ...
    'NumberTitle', 'off', ...
    'Color', 'w');

%title('360° LiDAR Scan of the Indoor Environment at Initial UAV Position');

axes_handle = axes;
hold(axes_handle, 'on');
grid(axes_handle, 'on');
axis(axes_handle, 'equal');

set(axes_handle, ...
    'Color', 'w', ...
    'XColor', 'k', ...
    'YColor', 'k', ...
    'GridColor', [0.75 0.75 0.75], ...
    'GridAlpha', 0.5);

xlim(axes_handle, [0 room_width]);
ylim(axes_handle, [0 room_height]);

xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Animated 2D LiDAR Scan');


% ==============================================================
% CELL 1 : DRAW ROOM
% ==============================================================

rectangle(axes_handle, ...
    'Position', [0 0 room_width room_height], ...
    'EdgeColor', 'k', ...
    'LineWidth', 2, ...
    'HandleVisibility', 'off');


% ==============================================================
% CELL 2 : DRAW ACTUAL OBSTACLES
% ==============================================================

theta = linspace(0, 2*pi, 200);

for obstacle_number = 1:size(obstacles, 1)

    obstacle_x = obstacles(obstacle_number, 1);
    obstacle_y = obstacles(obstacle_number, 2);
    obstacle_radius = obstacles(obstacle_number, 3);

    circle_x = obstacle_x + obstacle_radius*cos(theta);
    circle_y = obstacle_y + obstacle_radius*sin(theta);

    fill(axes_handle, circle_x, circle_y, ...
        [0.85 0.25 0.25], ...
        'EdgeColor', 'k', ...
        'HandleVisibility', 'off');

end


% ==============================================================
% CELL 3 : INITIALIZE LIDAR GRAPHICS
% ==============================================================

% Previously detected beams

detected_x = [];
detected_y = [];

% Previously maximum-range beams

maximum_range_x = [];
maximum_range_y = [];

% All detected endpoints

endpoint_x = [];
endpoint_y = [];


% Detected beam graphics

h_detected = plot(axes_handle, nan, nan, ...
    'Color', [0.00 0.65 0.15], ...
    'LineWidth', 0.8);

% Maximum-range beam graphics

h_maximum_range = plot(axes_handle, nan, nan, ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 0.6);

% Current active beam

h_current = plot(axes_handle, nan, nan, ...
    'Color', [0.00 0.65 1.00], ...
    'LineWidth', 1.8);

% Detected endpoints

h_endpoints = plot(axes_handle, nan, nan, ...
    '.', ...
    'Color', [0.00 0.65 0.15], ...
    'MarkerSize', 5);

% LiDAR position

h_lidar = plot(axes_handle, ...
    lidar_position(1), lidar_position(2), ...
    'bo', ...
    'MarkerFaceColor', 'b', ...
    'MarkerSize', 9);


% ==============================================================
% CELL 4 : CREATE LEGEND HANDLES
% ==============================================================

h_room_legend = plot(axes_handle, nan, nan, ...
    'k-', 'LineWidth', 2);

h_obstacle_legend = plot(axes_handle, nan, nan, ...
    'o', ...
    'MarkerFaceColor', [0.85 0.25 0.25], ...
    'MarkerEdgeColor', 'k', ...
    'MarkerSize', 8);

h_detected_legend = plot(axes_handle, nan, nan, ...
    '-', ...
    'Color', [0.00 0.65 0.15], ...
    'LineWidth', 1.5);

h_maximum_legend = plot(axes_handle, nan, nan, ...
    '-', ...
    'Color', [0.65 0.65 0.65], ...
    'LineWidth', 1.5);

h_current_legend = plot(axes_handle, nan, nan, ...
    '-', ...
    'Color', [0.00 0.65 1.00], ...
    'LineWidth', 1.8);

h_lidar_legend = plot(axes_handle, nan, nan, ...
    'bo', ...
    'MarkerFaceColor', 'b', ...
    'MarkerSize', 9);

legend(axes_handle, ...
    [h_room_legend, h_obstacle_legend, ...
     h_detected_legend, h_maximum_legend, ...
     h_current_legend, h_lidar_legend], ...
    {'Room boundary', 'Actual obstacle', ...
     'Detected beam', 'Maximum-range beam', ...
     'Current beam', 'LiDAR position'}, ...
    'Location', 'eastoutside');


% ==============================================================
% CELL 5 : ANIMATE LIDAR SCAN
% ==============================================================

number_of_beams = length(lidar_angles);

for beam_number = 1:number_of_beams

    endpoint = hit_points(beam_number, :);

    % ----------------------------------------------------------
    % Update current beam
    % ----------------------------------------------------------

    set(h_current, ...
        'XData', [lidar_position(1), endpoint(1)], ...
        'YData', [lidar_position(2), endpoint(2)]);


    % ----------------------------------------------------------
    % Store completed beam
    % ----------------------------------------------------------

    if hit_mask(beam_number)

        detected_x = [detected_x, ...
            lidar_position(1), endpoint(1), nan];

        detected_y = [detected_y, ...
            lidar_position(2), endpoint(2), nan];

        endpoint_x = [endpoint_x, endpoint(1)];
        endpoint_y = [endpoint_y, endpoint(2)];

    else

        maximum_range_x = [maximum_range_x, ...
            lidar_position(1), endpoint(1), nan];

        maximum_range_y = [maximum_range_y, ...
            lidar_position(2), endpoint(2), nan];

    end


    % ----------------------------------------------------------
    % Update completed beams
    % ----------------------------------------------------------

    set(h_detected, ...
        'XData', detected_x, ...
        'YData', detected_y);

    set(h_maximum_range, ...
        'XData', maximum_range_x, ...
        'YData', maximum_range_y);

    set(h_endpoints, ...
        'XData', endpoint_x, ...
        'YData', endpoint_y);


    % Update animation

    title(axes_handle, ...
        sprintf('Animated 2D LiDAR Scan: %d / %d beams', ...
        beam_number, number_of_beams));

    drawnow;

    pause(animation_delay);

end


% ==============================================================
% CELL 6 : FINISH ANIMATION
% ==============================================================

set(h_current, 'XData', nan, 'YData', nan);

title(axes_handle, ...
    sprintf('LiDAR Scan Complete: %d beams', number_of_beams));

drawnow;

end
