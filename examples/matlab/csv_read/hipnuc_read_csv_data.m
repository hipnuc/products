% IMU Data Visualization Script
% Copyright (c) 2024 HiPNUC (www.hipnuc.com)
% This script visualizes IMU sensor data from HiPNUC IMU products
% Support: https://www.hipnuc.com/contact.html
%
% Features:
% - Accelerometer data visualization (3-axis)
% - Gyroscope data visualization (3-axis)
% - Magnetometer data visualization (3-axis)
% - Euler angles visualization (Roll/Pitch/Yaw)

%% Clear workspace and command window
clear all;
close all;
clc;

%% Read CSV file
% Define file name - replace with your actual file path
filename = 'example_data.csv';

% Read header first to get column names
header = readtable(filename, 'Range', '1:1');
column_names = header.Properties.VariableNames(2:end); % Skip first column

% Read all data (including header)
data = readtable(filename, 'HeaderLines', 0);
data.Properties.VariableNames = ['frame_type', column_names];

%% Filter HI91 frames for plotting
hi91_data = data(strcmp(data.frame_type, 'HI91'), :);

%% Extract time information
% Convert system time to seconds from start
time = (hi91_data.sys_time - hi91_data.sys_time(1)) / 1000; % Convert to seconds

%% Calculate time differences and check for frame drops
time_diff = diff(hi91_data.sys_time) / 1000; % in seconds

% Calculate statistics for time differences
mean_diff = mean(time_diff);
std_diff = std(time_diff);
max_diff = max(time_diff);
min_diff = min(time_diff);

% Print time difference statistics to console
fprintf('Time difference statistics:\n');
fprintf('Mean: %.4f s\n', mean_diff);
fprintf('Std: %.4f s\n', std_diff);
fprintf('Min: %.4f s\n', min_diff);
fprintf('Max: %.4f s\n', max_diff);

% Identify potential frame drops (if time difference is significantly larger than mean)
threshold = mean_diff + 3 * std_diff; % 3 sigma threshold
potential_drops = find(time_diff > threshold);
if ~isempty(potential_drops)
    fprintf('\nPotential frame drops detected at indices:\n');
    for i = 1:length(potential_drops)
        fprintf('Between frames %d and %d, time diff: %.4f s\n', potential_drops(i), potential_drops(i)+1, time_diff(potential_drops(i)));
    end
else
    fprintf('\nNo potential frame drops detected.\n');
end

%% Create a figure with subplots for sensor data
figure('Name', 'IMU Data Visualization', 'Position', [100 100 1000 800]);

% Subplot for accelerometer data
subplot(2, 2, 1);
plot(time, hi91_data.acc_x, 'r.-');
hold on;
plot(time, hi91_data.acc_y, 'g.-');
plot(time, hi91_data.acc_z, 'b.-');
grid on;
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

% Subplot for gyroscope data
subplot(2, 2, 2);
plot(time, hi91_data.gyr_x, 'r.-');
hold on;
plot(time, hi91_data.gyr_y, 'g.-');
plot(time, hi91_data.gyr_z, 'b.-');
grid on;
xlabel('Time (s)');
ylabel('Angular Rate (deg/s)');
title('Gyroscope Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

% Subplot for magnetometer data
subplot(2, 2, 3);
plot(time, hi91_data.mag_x, 'r.-');
hold on;
plot(time, hi91_data.mag_y, 'g.-');
plot(time, hi91_data.mag_z, 'b.-');
grid on;
xlabel('Time (s)');
ylabel('Magnetic Field (uT)');
title('Magnetometer Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

% Subplot for Euler angles
subplot(2, 2, 4);
plot(time, hi91_data.roll, 'r.-');
hold on;
plot(time, hi91_data.pitch, 'g.-');
plot(time, hi91_data.yaw, 'b.-');
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Euler Angles');
legend('Roll', 'Pitch', 'Yaw');
