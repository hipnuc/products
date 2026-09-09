function result = analyze_imu(filename, show_plots)
%ANALYZE_IMU Illustrate Allan deviation and 10-second moving-mean statistics.
% Example: result = analyze_imu('sample_hi91.csv') or analyze_imu('samples.jsonl')
% Pass false as the second argument to return results without opening figures.
% Use static, regularly sampled data. No interpolation or resampling is done.
% These statistics are examples, not a standards-compliance assessment.

if nargin < 2
    show_plots = true;
end
validateattributes(show_plots, {'logical'}, {'scalar'});
data = read_hipnuc_recording(filename);
samples = height(data);
if samples < 3
    error('hipnuc:InsufficientData', 'Analysis needs at least three measurements.');
end
time_s = (data.sys_time - data.sys_time(1)) / 1000;
dt = time_s(end) / (samples - 1);
% HI91 time is quantized to milliseconds. Accept that quantization, but do
% not turn a recording with gaps or changing output rate into uniform data.
expected_time = (0:samples-1)' * dt;
if any(abs(time_s - expected_time) > 0.001 + 16 * eps(time_s(end)))
    error('hipnuc:IrregularTime', 'Sampling is not regular within the 1 ms timestamp resolution; use a continuous fixed-rate recording.');
end

% Keep CSV units at the loader boundary; convert explicitly for these plots.
gyro = [data.gyr_x, data.gyr_y, data.gyr_z] * 3600; % deg/h
accel = [data.acc_x, data.acc_y, data.acc_z] * 1e6; % micro-G
measurements = [gyro, accel];
[deviation, tau] = allan_deviation(measurements, dt);
[minimum, index] = min(deviation, [], 1);

% Keep the central hour for the 10 s statistic, or the whole shorter file.
segment_count = min(samples, round(3600 / dt));
first = floor((samples - segment_count) / 2) + 1;
last = first + segment_count - 1;
if dt > 10
    error('hipnuc:InsufficientRate', 'The 10 s statistic requires a sample period no longer than 10 seconds.');
end
window = round(10 / dt);
if segment_count < window + 1
    error('hipnuc:InsufficientData', 'The 10 s statistic needs at least two complete sliding windows; record for more than 10 seconds.');
end
moving_mean = movmean(measurements(first:last, :), window, 1, 'Endpoints', 'discard');
moving_std = std(moving_mean, 0, 1);

result.sample_count = samples;
result.sample_period_s = dt;
result.duration_s = time_s(end);
result.tau_s = tau;
result.gyro_allan_deg_h = deviation(:, 1:3);
result.accel_allan_ug = deviation(:, 4:6);
result.statistics_interval_s = [time_s(first), time_s(last)];
result.moving_window_s = window * dt;
axis_names = {'Gyro X'; 'Gyro Y'; 'Gyro Z'; 'Accel X'; 'Accel Y'; 'Accel Z'};
units = {'deg/h'; 'deg/h'; 'deg/h'; 'micro-G'; 'micro-G'; 'micro-G'};
result.summary = table(axis_names, units, minimum', tau(index), moving_std', ...
    'VariableNames', {'Axis', 'Unit', 'MinAllanDeviation', 'MinAllanTau_s', 'StdOf10sMovingMean'});
result.figures = [];
if show_plots
    result.figures = [plot_allan(tau, deviation(:, 1:3), 'Gyroscope', 'deg/h'), ...
        plot_allan(tau, deviation(:, 4:6), 'Accelerometer', 'micro-G')];
    fprintf('Samples: %d; period: %.9g s; duration: %.3f s\n', samples, dt, time_s(end));
    fprintf('Moving-mean statistic: %.3f to %.3f s; window %.6g s\n', ...
        time_s(first), time_s(last), result.moving_window_s);
    disp(result.summary);
end
end

function [deviation, tau] = allan_deviation(data, dt)
% Non-overlapping, equal-size cluster means; discard the incomplete tail.
% Averaging times are the actual integer cluster sizes times the sample period.
count = size(data, 1);
largest = floor(count / 3);
sizes = unique(round(logspace(0, log10(largest), 100)))';
tau = sizes * dt;
deviation = zeros(numel(sizes), size(data, 2));
integral = [zeros(1, size(data, 2)); cumsum(data, 1)];
for i = 1:numel(sizes)
    width = sizes(i);
    edges = (0:floor(count / width))' * width + 1;
    means = diff(integral(edges, :), 1, 1) / width;
    deviation(i, :) = sqrt(mean(diff(means, 1, 1).^2, 1) / 2);
end
end

function fig = plot_allan(tau, deviation, title_text, unit)
fig = figure('Name', [title_text, ' Allan deviation']);
loglog(tau, deviation);
xlabel('Averaging time (s)');
ylabel(['Allan deviation (', unit, ')']);
title(title_text);
legend('X', 'Y', 'Z');
grid on;
end
