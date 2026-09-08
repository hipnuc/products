function fig = plot_imu(filename)
%PLOT_IMU Plot acceleration, angular velocity, magnetic field and attitude.
% Example: plot_imu('sample_hi91.csv')
% Return the new figure handle; existing figures and workspace are untouched.

data = read_hi91_csv(filename);
required = {'mag_x', 'mag_y', 'mag_z', 'roll', 'pitch', 'imu_yaw'};
if ~all(ismember(required, data.Properties.VariableNames))
    error('hipnuc:MissingColumns', 'Plotting requires magnetic field and roll/pitch/imu_yaw columns.');
end
time_s = (data.sys_time - data.sys_time(1)) / 1000;
fig = figure('Name', 'HiPNUC HI91 measurements');
subplot(2, 2, 1);
plot(time_s, [data.acc_x, data.acc_y, data.acc_z]);
ylabel('Acceleration (G)');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
grid on;

subplot(2, 2, 2);
plot(time_s, [data.gyr_x, data.gyr_y, data.gyr_z]);
ylabel('Angular velocity (deg/s)');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
grid on;

subplot(2, 2, 3);
plot(time_s, [data.mag_x, data.mag_y, data.mag_z]);
ylabel('Magnetic field (uT)');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
grid on;

subplot(2, 2, 4);
plot(time_s, [data.roll, data.pitch, data.imu_yaw]);
ylabel('Angle (deg)');
legend('Roll', 'Pitch', 'Yaw');
xlabel('Time (s)');
grid on;
end
