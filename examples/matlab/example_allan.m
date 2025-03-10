clear;
clc;
close all;

%读取数据
imudata = readtable('D:\WANGFENG\CHAOHE\040数据\040-1.csv'); 

R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.80665;  % 标准重力加速度 (m/s^2)

% 单位转换
acc = [imudata.AccX, imudata.AccY, imudata.AccZ];
gyr = [imudata.GyrX, imudata.GyrY, imudata.GyrZ];

% 计算并打印数据信息
total_samples = length(acc);
total_time = (imudata.TimeStamp(end) - imudata.TimeStamp(1)) / 1000; % 总时间（秒）
imu_dt = mean(diff(imudata.TimeStamp)) / 1000;
Fs = 1 / imu_dt; % 采样频率


%% 数据信息
fprintf('数据信息:\n');
fprintf('  总样本数: %d\n', total_samples);
fprintf('  采样频率: %.2f Hz\n', Fs);
fprintf('  总采样时间: %.2f 秒 (%.2f h)\n', total_time, total_time/3600);
fprintf('  平均采样间隔: %.6f 秒\n\n', imu_dt);

axis_names = {'X', 'Y', 'Z'};

%% 数据范围信息
fprintf('数据范围:\n');
fprintf('  加速度计 (G):\n');
for i = 1:3
    fprintf('    %s轴: [%.6f, %.6f]\n', axis_names{i}, min(acc(:,i)), max(acc(:,i)));
end
fprintf('  陀螺仪 (deg/s):\n');
for i = 1:3
    fprintf('    %s轴: [%.6f, %.6f]\n', axis_names{i}, min(gyr(:,i)), max(gyr(:,i)));
end
fprintf('\n');

% 定义tau值（使用整数倍的采样周期）
m = logspace(0, log10(floor(length(gyr)/2)), 1000);
m = unique(round(m));  % 确保m是唯一的整数值
tau = m / 100;

% 创建图形
figure('Name', 'Gyroscope Allan Deviation');
figure('Name', 'Accelerometer Allan Deviation');
colors = { 'r', 'g', 'b'};
for i = 1:3
    % 陀螺仪Allan方差分析
    [avar_gyr, tau_gyr] = allanvar(gyr(:,i), m, Fs);
    % 加速度计Allan方差分析
    [avar_acc, tau_acc] = allanvar(acc(:,i), m, Fs);

    % 绘制陀螺仪Allan偏差曲线
    figure(1);
    loglog(tau_gyr, sqrt(avar_gyr), colors{i}, 'LineWidth', 2);
    hold on;

    % 绘制加速度计Allan偏差曲线
    figure(2);
    loglog(tau_acc, sqrt(avar_acc), colors{i}, 'LineWidth', 2);
    hold on;

    % 计算一些关键参数
    % 陀螺仪
    % 找到 tau_gyr 小于 1000 的索引
    index = tau_gyr < 1e3;

    % 使用这些索引找到对应的 avar_gyr 值
    avar_gyr_filtered = avar_gyr(index);
    tau_gyr_filtered = tau_gyr(index);

    % 查找过滤后的 avar_gyr 的最小值
    [min_avar, min_index] = min(avar_gyr_filtered);
    min_tau = tau_gyr_filtered(min_index);
    
    B_gyr = sqrt(min_avar);   % 零偏不稳定性 (deg/s)

    % 加速度计
    % 使用这些索引找到对应的 avar_gyr 值
    avar_acc_filtered = avar_acc(index);
    tau_acc_filtered = tau_acc(index);

    % 查找过滤后的 avar_gyr 的最小值
    [min_avar, min_index] = min(avar_acc_filtered);
    
    VRW = sqrt(avar_acc(1)) * 60;  % 速度随机游走 (G/sqrt(hr))
    B_acc = sqrt(min_avar);   % 零偏不稳定性 (G)

    % 输出结果
    fprintf('Results for %s Axis:\n', axis_names{i});
    fprintf('Gyroscope (Allan Variance):\n');
    fprintf('  零偏不稳定性: %.6f deg/h\n', B_gyr * 3600);
    fprintf('Accelerometer (Allan Variance):\n');
    fprintf('  零偏不稳定性: %.6f uG\n', B_acc * 1e6);
 %{   
    fprintf('Results for %s Axis:\n', axis_names{i});
    fprintf('Gyroscope (Allan Variance):\n');
    fprintf('  随机游走: %.6f deg/h\n', ARW);
    fprintf('Accelerometer (Allan Variance):\n');
    fprintf('  随机游走: %.6f m/s/sqrt(h)\n', VRW * 9.8);
%}
end
axis_temp = {'1', '2', '3'};
figure(1);
%title('Root Allan Variance of gyro');
title('Gyroscope Allan Deviation - All Axex');
xlabel('Averaging time,τ (s)');
ylabel('Allan Deviation (deg/h)');
legend(axis_names);
grid on;
hold off;

figure(2);
title('Accelerometer Allan Deviation - All Axes');
xlabel('Averaging time,τ (s)');
ylabel('Allan Deviation (mg)');
legend(axis_names);
grid on;
hold off;

%% 国军标10s平滑分析
% http://i2nav.cn/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=43bf2e0076d947aba8b58725ad8f5c15
% 国军标测算值：对惯导的实际表现有比较直接的影响，有现实指导意义；
smooth_time = 10; % 10秒平滑
samples_per_smooth = round(smooth_time * Fs);

fprintf('国军标10s平滑分析结果:\n');
for i = 1:3
    % 陀螺仪10s平滑
    gyr_smoothed = smooth(gyr(:,i), samples_per_smooth);

    % 加速度计10s平滑
    acc_smoothed = smooth(acc(:,i), samples_per_smooth);

    % 计算平滑后的标准差
    gyr_std = std(gyr_smoothed);
    acc_std = std(acc_smoothed);

    % 输出结果
    fprintf('%s轴:\n', axis_names{i});
    fprintf('  陀螺仪零偏稳定性: %.6f deg/h\n', gyr_std * 3600);
    fprintf('  加速度计零偏稳定性: %.6f uG\n\n', acc_std * 1e6);
end

