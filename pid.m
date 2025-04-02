% Clear workspace
clear; clc;

% Simulation parameters
max_speed = 50;       % Maximum angular speed (degrees/sec)
time_step = 0.01;     % Time step (10ms)
max_time = 100;        % Maximum simulation time (30 seconds)

% Get target angle from user input
target_angle = input('Enter the target angle (degrees): ');

% PID constants sets (5 different configurations)
pid_constants = [
    1, 0.1, 0.5;   % Set 1
    1, 0.1, 0.5;     % Set 2
    1.5, 0.2, 0.5;   % Set 3
    0.5, 0.05, 0.2; % Set 4
    1.5, 0.15, 0.8; % Set 5
];

% Define colors for plotting
colors = lines(5); % Generate 5 distinct colors

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
        control_output = Kp * error + Ki * integral + Kd * derivative;

        % Update speed based on PID control output
        speed = max(min(control_output, max_speed), -max_speed);

        % Update motor position
        current_angle = mod(current_angle + speed * time_step, 360); % Wrap position 0-360

        % Store data for plotting
        time_data = [time_data, t];
        speed_data = [speed_data, speed];
        angle_data = [angle_data, current_angle];

        % Update previous error
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
