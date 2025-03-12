function [allan_results, gjb_results, data_info] = allan_analysis(varargin)
% IMU Allan Variance Analysis
% This script performs Allan variance analysis on IMU data
% to determine bias instability characteristics
%
% 用法:
%   1. 作为独立脚本: allan_analysis
%      - 使用默认文件路径
%   2. 指定文件路径: allan_analysis('hipnuc_static_data/example.mat')
%   3. 作为函数调用: [allan_results, gjb_results, data_info] = allan_analysis(...)

% 处理输入参数
if nargin >= 1
    file_path = varargin{1};
else
    % 默认文件路径
    file_path = 'hipnuc_static_data/1xb_2000dps_12g_10h_1.mat';
end

% 检查是否作为脚本或函数运行
is_script = (nargout == 0);

% 如果作为脚本运行，只清除其他变量，保留file_path
if is_script
    % 保存file_path
    temp_path = file_path;
    
    % 清理环境，但不使用clear all，以避免清除函数
    close all;
    clc;
    
    % 恢复file_path
    file_path = temp_path;
end

%% 加载和准备数据
data = load(file_path);
[~, base_name, ~] = fileparts(file_path);

% 提取传感器数据
acc = [data.imudata.AccX, data.imudata.AccY, data.imudata.AccZ];
gyr = [data.imudata.GyrX, data.imudata.GyrY, data.imudata.GyrZ];

% 计算时间信息
total_samples = length(acc);
total_time = (data.imudata.TimeStamp(end) - data.imudata.TimeStamp(1)) / 1000; % seconds
imu_dt = mean(diff(data.imudata.TimeStamp)) / 1000;
Fs = 1 / imu_dt; % sampling frequency

% 准备数据信息结构体
info = struct();
info.file_name = base_name;
info.file_path = file_path;
info.total_samples = total_samples;
info.sampling_rate = Fs;
info.total_time_seconds = total_time;
info.total_time_hours = total_time/3600;
info.sampling_interval = imu_dt;
info.analysis_date = datestr(now);

% 如果存在metadata.description则添加
if isfield(data, 'metadata') && isfield(data.metadata, 'description')
    info.description = data.metadata.description;
else
    info.description = 'No description available';
end

% 添加数据范围信息
info.acc_range = struct();
info.gyr_range = struct();
for i = 1:3
    axis_name = ['axis_', char('X' + i - 1)];
    info.acc_range.(axis_name) = [min(acc(:,i)), max(acc(:,i))];
    info.gyr_range.(axis_name) = [min(gyr(:,i)), max(gyr(:,i))];
end

%% 显示数据集信息
fprintf('Dataset Information:\n');
fprintf('  Description: %s\n', info.description);
fprintf('  File: %s\n', base_name);
fprintf('  Total samples: %d\n', total_samples);
fprintf('  Sampling rate: %.2f Hz\n', Fs);
fprintf('  Total duration: %.2f s (%.2f h)\n', total_time, total_time/3600);
fprintf('  Average sampling interval: %f s\n\n', imu_dt);

axis_names = {'X', 'Y', 'Z'};

%% 显示数据范围信息
fprintf('Data Range:\n');
fprintf('  Accelerometer (G):\n');
for i = 1:3
    fprintf('    %s-axis: [%.6f, %.6f]\n', axis_names{i}, min(acc(:,i)), max(acc(:,i)));
end
fprintf('  Gyroscope (deg/s):\n');
for i = 1:3
    fprintf('    %s-axis: [%.6f, %.6f]\n', axis_names{i}, min(gyr(:,i)), max(gyr(:,i)));
end
fprintf('\n');

%% 计算Allan方差并创建图表
% 创建一个大图形,包含子图
figure('Name', ['IMU Allan Deviation Analysis - ', base_name], 'Position', [100, 100, 1200, 800]);

colors = {'r', 'g', 'b'};

% 存储结果的数组
gyr_bi = zeros(3, 1);
acc_bi = zeros(3, 1);
gyr_bi_tau = zeros(3, 1);
acc_bi_tau = zeros(3, 1);
numPoints = 100;

% 陀螺仪子图
subplot(2, 1, 1);
hold on;

% 存储Allan方差数据
allan_data = struct();
allan_data.gyro = cell(3, 1);
allan_data.accel = cell(3, 1);

for i = 1:3
    % 计算陀螺仪Allan方差 (deg/h)
    [avar_gyr, tau_gyr] = allanvar(gyr(:,i)*3600, imu_dt, numPoints);
    
    % 存储Allan方差数据
    allan_data.gyro{i}.avar = avar_gyr;
    allan_data.gyro{i}.tau = tau_gyr;
    
    % 绘制陀螺仪Allan偏差
    loglog(tau_gyr, avar_gyr, colors{i});
    
    % 寻找偏置不稳定性(Allan偏差的最小点)
    % 过滤数据到合理范围(< 1000s)
    index = tau_gyr < 1e3;
    avar_gyr_filtered = avar_gyr(index);
    tau_gyr_filtered = tau_gyr(index);
    [min_avar_gyr, min_idx_gyr] = min(avar_gyr_filtered);
    min_tau_gyr = tau_gyr_filtered(min_idx_gyr);
    
    % 存储结果
    gyr_bi(i) = min_avar_gyr;
    gyr_bi_tau(i) = min_tau_gyr;
end

title(['Gyroscope Allan Deviation - ', base_name]);
xlabel('Averaging time, τ (s)');
ylabel('Allan Deviation (deg/h)');
legend(axis_names);
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');
hold off;

% 加速度计子图
subplot(2, 1, 2);
hold on;

for i = 1:3
    % 计算加速度计Allan方差 (μg)
    [avar_acc, tau_acc] = allanvar(acc(:,i)*1e6, imu_dt, numPoints);
    
    % 存储Allan方差数据
    allan_data.accel{i}.avar = avar_acc;
    allan_data.accel{i}.tau = tau_acc;
    
    % 绘制加速度计Allan偏差
    loglog(tau_acc, avar_acc, colors{i});
    
    % 加速度计偏置不稳定性
    index = tau_acc < 1e3;
    avar_acc_filtered = avar_acc(index);
    tau_acc_filtered = tau_acc(index);
    [min_avar_acc, min_idx_acc] = min(avar_acc_filtered);
    min_tau_acc = tau_acc_filtered(min_idx_acc);
    
    % 存储结果
    acc_bi(i) = min_avar_acc;
    acc_bi_tau(i) = min_tau_acc;
end

title(['Accelerometer Allan Deviation - ', base_name]);
xlabel('Averaging time, τ (s)');
ylabel('Allan Deviation (μg)');
legend(axis_names);
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');
hold off;

% 保存Allan方差结果到info结构体
info.allan_data = allan_data;

%% 显示结果分组
fprintf('Allan Variance Analysis Results:\n\n');

% 陀螺仪结果
fprintf('Gyroscope Bias Instability:\n');
for i = 1:3
    fprintf('  %s-axis: %.6f deg/h at %.1f s\n', axis_names{i}, gyr_bi(i), gyr_bi_tau(i));
end
fprintf('\n');

% 加速度计结果
fprintf('Accelerometer Bias Instability:\n');
for i = 1:3
    fprintf('  %s-axis: %.6f μg at %.1f s\n', axis_names{i}, acc_bi(i), acc_bi_tau(i));
end
fprintf('\n');

%% GJB 10s稳定性 - 使用中间1小时数据
fprintf('GJB 10s稳定性:\n');

% 计算中间1小时数据的起始和结束索引
one_hour_samples = round(1 * 3600 * Fs);
total_available_samples = total_samples;

% 确保有足够的数据
if total_available_samples < one_hour_samples
    gjb_sample_count = total_available_samples;
    start_idx = 1;
else
    gjb_sample_count = one_hour_samples;
    % 计算中间段的起始索引
    start_idx = max(1, round((total_available_samples - gjb_sample_count) / 2));
end
end_idx = start_idx + gjb_sample_count - 1;
end_idx = min(end_idx, total_available_samples);

% 提取中间段数据
acc_gjb = acc(start_idx:end_idx, :);
gyr_gjb = gyr(start_idx:end_idx, :);

fprintf('使用数据段: 从 %.2f 小时到 %.2f 小时 (共 %.2f 小时)\n',  (start_idx-1)/Fs/3600, end_idx/Fs/3600, (end_idx-start_idx+1)/Fs/3600)

% 陀螺仪10s稳定性
fprintf('Gyroscope:\n');
gyr_std = zeros(3, 1);
for i = 1:3
    % 转换为deg/h
    gyr_std(i) = gjb_10s_stability(gyr_gjb(:,i), Fs) * 3600;
    fprintf('  %s-axis: %.6f deg/h\n', axis_names{i}, gyr_std(i));
end
fprintf('\n');

% 加速度计10s稳定性
fprintf('Accelerometer:\n');
acc_std = zeros(3, 1);
for i = 1:3
    % 转换为μg
    acc_std(i) = gjb_10s_stability(acc_gjb(:,i), Fs) * 1e6;
    fprintf('  %s-axis: %.6f μg\n', axis_names{i}, acc_std(i));
end
fprintf('\n');

%% 创建结果表格和信息结构体(仅当作为函数调用时)
if ~is_script
    % 创建Allan方差结果表格
    allan_results = table();
    allan_results.FileName = repmat({base_name}, 6, 1);
    allan_results.SensorType = [repmat({'Gyro'}, 3, 1); repmat({'Accel'}, 3, 1)];
    allan_results.Axis = [axis_names'; axis_names'];
    allan_results.BiasInstability = [gyr_bi; acc_bi];
    allan_results.BiasInstabilityTau = [gyr_bi_tau; acc_bi_tau];
    allan_results.Unit = [repmat({'deg/h'}, 3, 1); repmat({'μg'}, 3, 1)];
    
    % 创建GJB 10s稳定性结果表格
    gjb_results = table();
    gjb_results.FileName = repmat({base_name}, 6, 1);
    gjb_results.SensorType = [repmat({'Gyro'}, 3, 1); repmat({'Accel'}, 3, 1)];
    gjb_results.Axis = [axis_names'; axis_names'];
    gjb_results.Stability10s = [gyr_std; acc_std];
    gjb_results.Unit = [repmat({'deg/h'}, 3, 1); repmat({'μg'}, 3, 1)];
    
    % 添加Allan方差和GJB稳定性结果到info结构体
    info.gyro_bias_instability = gyr_bi;
    info.gyro_bias_instability_tau = gyr_bi_tau;
    info.accel_bias_instability = acc_bi;
    info.accel_bias_instability_tau = acc_bi_tau;
    info.gyro_stability_10s = gyr_std;
    info.accel_stability_10s = acc_std;
    
    % 返回数据信息结构体
    data_info = info;
else
    % 如果作为脚本运行，只为了保持函数签名一致
    allan_results = [];
    gjb_results = [];
    data_info = [];
end

end
