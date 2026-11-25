
function [path, success, expanded_nodes, path_length] = ...
    astar(occupancy_grid, grid_x, grid_y, ...
    start_position, goal_position)

% ==============================================================
% A* GLOBAL PATH PLANNING
%
% Inputs:
% occupancy_grid : 0 = Free, 1 = Occupied
% grid_x         : X coordinates of grid cell centres
% grid_y         : Y coordinates of grid cell centres
% start_position : [X, Y]
% goal_position  : [X, Y]
%
% Outputs:
% path           : Planned path in world coordinates
% success        : 1 if path found, otherwise 0
% expanded_nodes : Number of expanded nodes
% path_length    : Total path length in metres
% ==============================================================


% ==============================================================
% 1. INITIALIZATION
% ==============================================================

[number_of_rows, number_of_columns] = size(occupancy_grid);

grid_resolution = grid_x(2) - grid_x(1);

path = [];

success = false;

expanded_nodes = 0;

path_length = inf;


% ==============================================================
% 2. CONVERT START AND GOAL TO GRID INDICES
% ==============================================================

start_column = floor(start_position(1) / grid_resolution) + 1;
start_row = floor(start_position(2) / grid_resolution) + 1;

goal_column = floor(goal_position(1) / grid_resolution) + 1;
goal_row = floor(goal_position(2) / grid_resolution) + 1;


% Check whether start and goal are inside the grid

if start_row < 1 || start_row > number_of_rows || ...
   start_column < 1 || start_column > number_of_columns

    warning('Start position is outside the occupancy grid.');
    return;

end


if goal_row < 1 || goal_row > number_of_rows || ...
   goal_column < 1 || goal_column > number_of_columns

    warning('Goal position is outside the occupancy grid.');
    return;

end


% Check whether start and goal are occupied

if occupancy_grid(start_row, start_column) == 1

    warning('Start position is inside an occupied cell.');
    return;

end


if occupancy_grid(goal_row, goal_column) == 1

    warning('Goal position is inside an occupied cell.');
    return;

end


% Convert grid coordinates to linear indices

start_index = sub2ind( ...
    [number_of_rows, number_of_columns], ...
    start_row, start_column);

goal_index = sub2ind( ...
    [number_of_rows, number_of_columns], ...
    goal_row, goal_column);


% ==============================================================
% 3. INITIALIZE A* ARRAYS
% ==============================================================

g_score = inf(number_of_rows, number_of_columns);

f_score = inf(number_of_rows, number_of_columns);

parent = zeros(number_of_rows, number_of_columns);

open_list = false(number_of_rows, number_of_columns);

closed_list = false(number_of_rows, number_of_columns);


% Starting node

g_score(start_index) = 0;

heuristic = sqrt( ...
    (goal_row - start_row)^2 + ...
    (goal_column - start_column)^2);

f_score(start_index) = heuristic;

open_list(start_index) = true;


% ==============================================================
% 4. DEFINE EIGHT POSSIBLE MOVEMENTS
% ==============================================================

% Each row represents [Change in row, Change in column]

movements = [
    -1,  0;
     1,  0;
     0, -1;
     0,  1;
    -1, -1;
    -1,  1;
     1, -1;
     1,  1
];


% ==============================================================
% 5. A* SEARCH
% ==============================================================

while any(open_list(:))


    % ----------------------------------------------------------
    % Find the open node having minimum f_score
    % ----------------------------------------------------------

    open_indices = find(open_list);

    [~, minimum_position] = min(f_score(open_indices));

    current_index = open_indices(minimum_position);


    % ----------------------------------------------------------
    % Check whether goal has been reached
    % ----------------------------------------------------------

    if current_index == goal_index

        success = true;

        break;

    end


    % ----------------------------------------------------------
    % Move current node from open list to closed list
    % ----------------------------------------------------------

    open_list(current_index) = false;

    closed_list(current_index) = true;

    expanded_nodes = expanded_nodes + 1;


    % Convert current linear index to row and column

    [current_row, current_column] = ...
        ind2sub([number_of_rows, number_of_columns], current_index);


    % ==========================================================
    % 6. CHECK ALL EIGHT NEIGHBOURS
    % ==========================================================

    for movement_number = 1:size(movements, 1)


        neighbour_row = current_row + movements(movement_number, 1);

        neighbour_column = current_column + movements(movement_number, 2);


        % ------------------------------------------------------
        % Check grid boundaries
        % ------------------------------------------------------

        if neighbour_row < 1 || neighbour_row > number_of_rows || ...
           neighbour_column < 1 || neighbour_column > number_of_columns

            continue;

        end


        % ------------------------------------------------------
        % Ignore occupied cells
        % ------------------------------------------------------

        if occupancy_grid(neighbour_row, neighbour_column) == 1

            continue;

        end


        % ------------------------------------------------------
        % Ignore already explored cells
        % ------------------------------------------------------

        if closed_list(neighbour_row, neighbour_column)

            continue;

        end


        % ------------------------------------------------------
        % Prevent diagonal movement through obstacle corners
        % ------------------------------------------------------

        row_change = movements(movement_number, 1);

        column_change = movements(movement_number, 2);

        if row_change ~= 0 && column_change ~= 0

            adjacent_cell_1 = occupancy_grid( ...
                current_row + row_change, current_column);

            adjacent_cell_2 = occupancy_grid( ...
                current_row, current_column + column_change);


            if adjacent_cell_1 == 1 || adjacent_cell_2 == 1

                continue;

            end

        end


        % ------------------------------------------------------
        % Calculate movement cost
        % ------------------------------------------------------

        if row_change ~= 0 && column_change ~= 0

            movement_cost = sqrt(2);

        else

            movement_cost = 1;

        end


        % ------------------------------------------------------
        % Calculate tentative g_score
        % ------------------------------------------------------

        tentative_g_score = ...
            g_score(current_index) + movement_cost;


        % ------------------------------------------------------
        % Update neighbour if a better path is found
        % ------------------------------------------------------

        if tentative_g_score < ...
                g_score(neighbour_row, neighbour_column)


            neighbour_index = sub2ind( ...
                [number_of_rows, number_of_columns], ...
                neighbour_row, neighbour_column);


            parent(neighbour_index) = current_index;


            g_score(neighbour_index) = tentative_g_score;


            % Euclidean distance heuristic

            heuristic = sqrt( ...
                (goal_row - neighbour_row)^2 + ...
                (goal_column - neighbour_column)^2);


            f_score(neighbour_index) = ...
                tentative_g_score + heuristic;


            open_list(neighbour_index) = true;

        end

    end

end


% ==============================================================
% 7. RECONSTRUCT THE PATH
% ==============================================================

if success


    path_indices = goal_index;

    current_index = goal_index;


    while current_index ~= start_index

        current_index = parent(current_index);

        if current_index == 0

            warning('Path reconstruction failed.');

            path = [];

            success = false;

            return;

        end

        path_indices = [current_index; path_indices];

    end


    % Convert linear indices to row and column indices

    [path_rows, path_columns] = ...
        ind2sub([number_of_rows, number_of_columns], path_indices);


    % Convert grid indices into world coordinates

    path_x = grid_x(path_columns);

    path_y = grid_y(path_rows);

    path = [path_x(:), path_y(:)];


    % Replace first and last grid centres with exact positions

    path(1, :) = start_position;

    path(end, :) = goal_position;


    % Calculate total path length

    path_differences = diff(path, 1, 1);

    segment_lengths = sqrt(sum(path_differences.^2, 2));

    path_length = sum(segment_lengths);

end

end
