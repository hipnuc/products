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

%% Create figure for accelerometer data
figure('Name', 'Accelerometer Data', 'Position', [100 100 800 400]);
plot(time, hi91_data.acc_x, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, hi91_data.acc_y, 'g-', 'LineWidth', 1.5);
plot(time, hi91_data.acc_z, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

%% Create figure for gyroscope data
figure('Name', 'Gyroscope Data', 'Position', [100 550 800 400]);
plot(time, hi91_data.gyr_x, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, hi91_data.gyr_y, 'g-', 'LineWidth', 1.5);
plot(time, hi91_data.gyr_z, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angular Rate (deg/s)');
title('Gyroscope Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

%% Create figure for magnetometer data
figure('Name', 'Magnetometer Data', 'Position', [950 100 800 400]);
plot(time, hi91_data.mag_x, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, hi91_data.mag_y, 'g-', 'LineWidth', 1.5);
plot(time, hi91_data.mag_z, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Magnetic Field (uT)');
title('Magnetometer Measurements');
legend('X-axis', 'Y-axis', 'Z-axis');

%% Create figure for Euler angles
figure('Name', 'Euler Angles', 'Position', [950 550 800 400]);
plot(time, hi91_data.roll, 'r-', 'LineWidth', 1.5);
hold on;
plot(time, hi91_data.pitch, 'g-', 'LineWidth', 1.5);
plot(time, hi91_data.yaw, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Euler Angles');
legend('Roll', 'Pitch', 'Yaw');


