
function [acceleration_command, integral_error] = ...
    pid_controller(target_position, current_position, ...
    current_velocity, integral_error, time_step, ...
    Kp, Ki, Kd)

% ==============================================================
% PID CONTROLLER
% 2D POSITION CONTROL
% ==============================================================


% STEP 1: CALCULATE POSITION ERROR

position_error = target_position - current_position;


% STEP 2: UPDATE INTEGRAL ERROR

integral_error = integral_error + position_error * time_step;


% STEP 3: CALCULATE PROPORTIONAL TERM

proportional_term = Kp * position_error;


% STEP 4: CALCULATE INTEGRAL TERM

integral_term = Ki * integral_error;


% STEP 5: CALCULATE DERIVATIVE TERM

derivative_term = -Kd * current_velocity;


% STEP 6: CALCULATE ACCELERATION COMMAND

acceleration_command = ...
    proportional_term + integral_term + derivative_term;

end
