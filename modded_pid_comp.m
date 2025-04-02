% Clear workspace
clear; clc;

% Simulation parameters
max_speed = 100;       % Maximum angular speed (degrees/sec)
time_step = 0.02;     % Time step (10ms)
max_time = 20;       % Maximum simulation time (30 seconds)

% Get target angle from user input
target_angle = input('Enter the target angle (degrees): ');

% PID constants sets (5 different configurations + 1 with dynamic Ki and fixed Kd)
pid_constants = [
    2.5, 0.02, 0.9;   % Set 1
    0.5, 1, 0.5;     % Set 2
    2.5, 0, 0.9;        % Set 6: Kp=1, Ki dynamic (second integral), Kd=0.4
];

% Define colors for plotting
colors = lines(3); % Generate 6 distinct colors (including the new set)

% Function to calculate wrapped error for angle (0-360 degrees)
wrap_error = @(target, current) mod(target - current + 180, 360) - 180;

% Create figure for the plots
figure;

% Subplot 1: Speed vs Time
subplot(2,1,1);
hold on;
title('Speed vs Time');
xlabel('Time (s)');
ylabel('Speed (deg/sec)');
grid on;

% Subplot 2: Angle vs Time
subplot(2,1,2);
hold on;
title('Angle vs Time');
xlabel('Time (s)');
ylabel('Angle (deg)');
grid on;

% Loop over each PID constant set
for i = 1:size(pid_constants, 1)
    Kp = pid_constants(i, 1);
    Ki = pid_constants(i, 2);
    Kd = pid_constants(i, 3);

    % Initial conditions
    current_angle = 0;
    previous_error = 0;
    integral = 0;
    speed = 0;
    integral_of_integral = 0;  % Variable to store the second-level integral for Ki

    % Data storage for this run
    time_data = [];
    speed_data = [];
    angle_data = [];

    % Simulation loop
    for t = 0:time_step:max_time
        % Compute error
        error = wrap_error(target_angle, current_angle);

        % PID calculations
        integral = integral + error * time_step;
        derivative = (error - previous_error) / time_step;

        % For Set 3, calculate Ki dynamically at each step by integrating the integral
        if i == 3
            fprintf('Time: %.2f s, integral: %.4f\n', t, integral);
            integral_of_integral = integral_of_integral + integral * time_step;   % Second-level integral for Ki
            fprintf('Time: %.2f s, integral_of_integral: %.4f\n', t, integral_of_integral);
            Ki = 1 / integral_of_integral;

            % Print Ki at every step for Set 3
            fprintf('Time: %.2f s, Ki: %.4f\n', t, Ki);
        end


        % Calculate control output
        control_output = Kp * error + Ki * integral + Kd * derivative;

        % Update speed based on PID control output
        speed = max(min(control_output, max_speed), -max_speed);

        % Update motor position
        current_angle = mod(current_angle + speed * time_step, 360); % Wrap position 0-360

        % Store data for plotting
        time_data = [time_data, t];
        speed_data = [speed_data, speed];
        angle_data = [angle_data, current_angle];

        % Update previous error for the next step
        previous_error = error;


        % Stop if close to target
        if abs(error) < 0.5 && abs(speed) < 0.1
            break;
        end
    end

    % Plot Speed vs Time
    subplot(2,1,1);
    plot(time_data, speed_data, 'LineWidth', 2, 'Color', colors(i,:), 'DisplayName', ['Kp=' num2str(Kp) ', Ki=' num2str(Ki) ', Kd=' num2str(Kd)]);

    % Plot Angle vs Time
    subplot(2,1,2);
    plot(time_data, angle_data, 'LineWidth', 2, 'Color', colors(i,:));
end

% Plot target angle as a horizontal line on the "Angle vs Time" graph
subplot(2,1,2);
hold on;
% Create a horizontal line using the plot function
plot([0 max_time], [target_angle target_angle], 'r', 'LineWidth', 2, 'DisplayName', 'Target Angle');

% Add legends
subplot(2,1,1);
legend('show');

subplot(2,1,2);
legend('show');

hold off;
