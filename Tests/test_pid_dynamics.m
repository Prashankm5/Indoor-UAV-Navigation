
% ==============================================================
% TEST : PID CONTROLLER + UAV DYNAMICS
% STAGE 6 : CLOSED-LOOP POINT-TO-POINT MOVEMENT
% ==============================================================

clear;
clc;
close all;

project_folder = pwd;
addpath(genpath(project_folder));


% STEP 1: INITIAL UAV STATE

position = [1.5, 1.5];

velocity = [0, 0];


% STEP 2: TARGET POSITION

target_position = [5.0, 5.0];


% STEP 3: SIMULATION PARAMETERS

time_step = 0.05;

maximum_time = 15;

maximum_steps = round(maximum_time / time_step);

maximum_acceleration = 2.0;

maximum_speed = 1.0;


% STEP 4: PID GAINS

Kp = 1.5;
Ki = 0;
Kd = 2.0;


% STEP 5: INITIALISE PID INTEGRAL ERROR

integral_error = [0, 0];


% STEP 6: INITIALISE TRAJECTORY STORAGE

trajectory = zeros(maximum_steps + 1, 2);

trajectory(1, :) = position;

time = zeros(maximum_steps + 1, 1);

reached_goal = false;


% STEP 7: SIMULATION LOOP

for step = 1:maximum_steps

    % Calculate acceleration command using PID

    [acceleration_command, integral_error] = ...
        pid_controller(target_position, position, ...
        velocity, integral_error, time_step, ...
        Kp, Ki, Kd);


    % Update UAV position and velocity

    [position, velocity, actual_acceleration] = ...
        update_uav(position, velocity, acceleration_command, ...
        time_step, maximum_acceleration, maximum_speed);


    % Store updated trajectory

    trajectory(step + 1, :) = position;

    time(step + 1) = step * time_step;


    % Calculate distance from target

    position_error = norm(target_position - position);

    speed = norm(velocity);


    % Check whether the UAV reaches the goal

    if position_error <= 0.05 && speed <= 0.05

        reached_goal = true;

    end

end


% STEP 8: REMOVE UNUSED STORAGE

trajectory = trajectory(1:step + 1, :);

time = time(1:step + 1);


% STEP 9: DISPLAY RESULTS

fprintf('\n============================================\n');
fprintf(' PID + UAV DYNAMICS TEST\n');
fprintf('============================================\n');

fprintf('Initial position  : [%.3f, %.3f] m\n', ...
    trajectory(1, 1), trajectory(1, 2));

fprintf('Target position   : [%.3f, %.3f] m\n', ...
    target_position(1), target_position(2));

fprintf('Final position    : [%.3f, %.3f] m\n', ...
    position(1), position(2));

fprintf('Final speed       : %.4f m/s\n', norm(velocity));

fprintf('Final position error : %.4f m\n', ...
    norm(target_position - position));

fprintf('Simulation time   : %.2f s\n', time(end));

fprintf('Simulation steps  : %d\n', step);

if reached_goal

    fprintf('Goal reached      : YES\n');

else

    fprintf('Goal reached      : NO\n');

end

fprintf('============================================\n');


% STEP 10: PLOT UAV TRAJECTORY

figure;

plot(trajectory(:, 1), trajectory(:, 2), ...
    'b-', 'LineWidth', 1.8);

hold on;

plot(trajectory(1, 1), trajectory(1, 2), ...
    'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9);

plot(target_position(1), target_position(2), ...
    'r*', 'MarkerSize', 12, 'LineWidth', 2);

plot(position(1), position(2), ...
    'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 7);


grid on;
axis equal;

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('PID Controlled UAV Movement');

legend('UAV trajectory', 'Start', ...
    'Target', 'Final position', ...
    'Location', 'best');

hold off;

fprintf('Final integral error : [%.4f, %.4f]\n', ...
    integral_error(1), integral_error(2));

fprintf('Final commanded acceleration : [%.6f, %.6f]\n', ...
    acceleration_command(1), acceleration_command(2));

fprintf('Final actual acceleration : [%.6f, %.6f]\n', ...
    actual_acceleration(1), actual_acceleration(2));



% ==============================================================
% PID FINAL STATE DIAGNOSTICS
% ==============================================================

format long g;

final_position_error = target_position - position;

proportional_term = Kp * final_position_error;

integral_term = Ki * integral_error;

derivative_term = -Kd * velocity;

calculated_acceleration = ...
    proportional_term + integral_term + derivative_term;


disp(' ');
disp('========== PID FINAL STATE DIAGNOSTICS ==========');

disp('Final position error:');
disp(final_position_error);

disp('Final velocity:');
disp(velocity);

disp('Final integral error:');
disp(integral_error);

disp('Proportional term:');
disp(proportional_term);

disp('Integral term:');
disp(integral_term);

disp('Derivative term:');
disp(derivative_term);

disp('Calculated acceleration:');
disp(calculated_acceleration);

disp('Previously stored acceleration command:');
disp(acceleration_command);

disp('=================================================');

