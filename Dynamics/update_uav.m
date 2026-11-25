
function [new_position, new_velocity, actual_acceleration] = ...
    update_uav(position, velocity, acceleration_command, ...
    time_step, maximum_acceleration, maximum_speed)

% ==============================================================
% UPDATE UAV
% 2D DOUBLE INTEGRATOR DYNAMICS
% ==============================================================


% STEP 1: LIMIT ACCELERATION

acceleration_magnitude = norm(acceleration_command);

if acceleration_magnitude > maximum_acceleration

    acceleration_command = ...
        acceleration_command / acceleration_magnitude ...
        * maximum_acceleration;

end


% STEP 2: UPDATE VELOCITY

new_velocity = velocity + acceleration_command * time_step;


% STEP 3: LIMIT VELOCITY

velocity_magnitude = norm(new_velocity);

if velocity_magnitude > maximum_speed

    new_velocity = ...
        new_velocity / velocity_magnitude * maximum_speed;

end


% STEP 4: UPDATE POSITION

new_position = position + new_velocity * time_step;


% STEP 5: STORE ACTUAL ACCELERATION

actual_acceleration = (new_velocity - velocity) / time_step;

end
