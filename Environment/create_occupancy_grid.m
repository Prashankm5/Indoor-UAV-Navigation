
function [occupancy_grid, grid_x, grid_y] = ...
    create_occupancy_grid(room_width, room_height, ...
    grid_resolution, uav_radius, safety_margin, obstacles)


% =============================================================
% Add project folders and all subfolders
% =============================================================
project_folder = pwd;

addpath(genpath(project_folder));
% ==============================================================
% CREATE OCCUPANCY GRID
%
% 0 = Free cell
% 1 = Occupied cell
% ==============================================================

% --------------------------------------------------------------
% 1. CREATE GRID
% --------------------------------------------------------------

number_of_columns = round(room_width / grid_resolution);
number_of_rows = round(room_height / grid_resolution);

grid_x = ((1:number_of_columns) - 0.5) * grid_resolution;  % x coordinates of centre of all grid.
%fprintf((1:number_of_columns) - 0.5);
grid_y = ((1:number_of_rows) - 0.5) * grid_resolution;

occupancy_grid = zeros(number_of_rows, number_of_columns);

% Half width of one square cell
half_cell = grid_resolution / 2;

% ==============================================================
% CELL 2 : UAV INFLATION
% ==============================================================

inflation_radius = uav_radius + safety_margin;


% ==============================================================
% CELL 3 : CHECK EVERY GRID CELL
% ==============================================================

for row = 1:number_of_rows

    for column = 1:number_of_columns

        % Centre of current grid cell

        x = grid_x(column);
        y = grid_y(row);


        % ======================================================
        % CHECK 1 : ROOM WALLS
        % ======================================================

        left_distance = x - half_cell;

        right_distance = room_width - (x + half_cell);

        bottom_distance = y - half_cell;

        top_distance = room_height - (y + half_cell);
        % || mean or and && means and
        if left_distance <= inflation_radius || ...
                right_distance <= inflation_radius || ...
                bottom_distance <= inflation_radius || ...
                top_distance <= inflation_radius

            occupancy_grid(row, column) = 1;

        end


        % ======================================================
        % CHECK 2 : CIRCULAR OBSTACLES
        % ======================================================
        % matlab starts from 1 not 0 like python
        for obstacle_index = 1:size(obstacles, 1)

            % Obstacle information

            obstacle_x = obstacles(obstacle_index, 1);
            obstacle_y = obstacles(obstacle_index, 2);
            obstacle_radius = obstacles(obstacle_index, 3);

            % Effective obstacle radius

            effective_radius = obstacle_radius + inflation_radius;

            % Minimum distance from obstacle centre to square cell

            dx = max(abs(obstacle_x - x) - half_cell, 0);

            dy = max(abs(obstacle_y - y) - half_cell, 0);

            minimum_distance = sqrt(dx^2 + dy^2);

            % Check whether circle touches or overlaps the cell

            if minimum_distance <= effective_radius

                occupancy_grid(row, column) = 1;

            end

        end

    end

end

end