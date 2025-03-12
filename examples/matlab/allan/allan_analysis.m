function [allan_results, gjb_results, data_info] = allan_analysis(varargin)
% IMU Allan Variance Analysis
% This script performs Allan variance analysis on IMU data
% to determine bias instability characteristics
%
% �÷�:
%   1. ��Ϊ�����ű�: allan_analysis
%      - ʹ��Ĭ���ļ�·��
%   2. ָ���ļ�·��: allan_analysis('hipnuc_static_data/example.mat')
%   3. ��Ϊ��������: [allan_results, gjb_results, data_info] = allan_analysis(...)

% �����������
if nargin >= 1
    file_path = varargin{1};
else
    % Ĭ���ļ�·��
    file_path = 'hipnuc_static_data/1xb_2000dps_12g_10h_1.mat';
end

% ����Ƿ���Ϊ�ű���������
is_script = (nargout == 0);

% �����Ϊ�ű����У�ֻ�����������������file_path
if is_script
    % ����file_path
    temp_path = file_path;
    
    % ������������ʹ��clear all���Ա����������
    close all;
    clc;
    
    % �ָ�file_path
    file_path = temp_path;
end

%% ���غ�׼������
data = load(file_path);
[~, base_name, ~] = fileparts(file_path);

% ��ȡ����������
acc = [data.imudata.AccX, data.imudata.AccY, data.imudata.AccZ];
gyr = [data.imudata.GyrX, data.imudata.GyrY, data.imudata.GyrZ];

% ����ʱ����Ϣ
total_samples = length(acc);
total_time = (data.imudata.TimeStamp(end) - data.imudata.TimeStamp(1)) / 1000; % seconds
imu_dt = mean(diff(data.imudata.TimeStamp)) / 1000;
Fs = 1 / imu_dt; % sampling frequency

% ׼��������Ϣ�ṹ��
info = struct();
info.file_name = base_name;
info.file_path = file_path;
info.total_samples = total_samples;
info.sampling_rate = Fs;
info.total_time_seconds = total_time;
info.total_time_hours = total_time/3600;
info.sampling_interval = imu_dt;
info.analysis_date = datestr(now);

% �������metadata.description�����
if isfield(data, 'metadata') && isfield(data.metadata, 'description')
    info.description = data.metadata.description;
else
    info.description = 'No description available';
end

% ������ݷ�Χ��Ϣ
info.acc_range = struct();
info.gyr_range = struct();
for i = 1:3
    axis_name = ['axis_', char('X' + i - 1)];
    info.acc_range.(axis_name) = [min(acc(:,i)), max(acc(:,i))];
    info.gyr_range.(axis_name) = [min(gyr(:,i)), max(gyr(:,i))];
end

%% ��ʾ���ݼ���Ϣ
fprintf('Dataset Information:\n');
fprintf('  Description: %s\n', info.description);
fprintf('  File: %s\n', base_name);
fprintf('  Total samples: %d\n', total_samples);
fprintf('  Sampling rate: %.2f Hz\n', Fs);
fprintf('  Total duration: %.2f s (%.2f h)\n', total_time, total_time/3600);
fprintf('  Average sampling interval: %f s\n\n', imu_dt);

axis_names = {'X', 'Y', 'Z'};

%% ��ʾ���ݷ�Χ��Ϣ
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

%% ����Allan�������ͼ��
% ����һ����ͼ��,������ͼ
figure('Name', ['IMU Allan Deviation Analysis - ', base_name], 'Position', [100, 100, 1200, 800]);

colors = {'r', 'g', 'b'};

% �洢���������
gyr_bi = zeros(3, 1);
acc_bi = zeros(3, 1);
gyr_bi_tau = zeros(3, 1);
acc_bi_tau = zeros(3, 1);
numPoints = 100;

% ��������ͼ
subplot(2, 1, 1);
hold on;

% �洢Allan��������
allan_data = struct();
allan_data.gyro = cell(3, 1);
allan_data.accel = cell(3, 1);

for i = 1:3
    % ����������Allan���� (deg/h)
    [avar_gyr, tau_gyr] = allanvar(gyr(:,i)*3600, imu_dt, numPoints);
    
    % �洢Allan��������
    allan_data.gyro{i}.avar = avar_gyr;
    allan_data.gyro{i}.tau = tau_gyr;
    
    % ����������Allanƫ��
    loglog(tau_gyr, avar_gyr, colors{i});
    
    % Ѱ��ƫ�ò��ȶ���(Allanƫ�����С��)
    % �������ݵ�����Χ(< 1000s)
    index = tau_gyr < 1e3;
    avar_gyr_filtered = avar_gyr(index);
    tau_gyr_filtered = tau_gyr(index);
    [min_avar_gyr, min_idx_gyr] = min(avar_gyr_filtered);
    min_tau_gyr = tau_gyr_filtered(min_idx_gyr);
    
    % �洢���
    gyr_bi(i) = min_avar_gyr;
    gyr_bi_tau(i) = min_tau_gyr;
end

title(['Gyroscope Allan Deviation - ', base_name]);
xlabel('Averaging time, �� (s)');
ylabel('Allan Deviation (deg/h)');
legend(axis_names);
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');
hold off;

% ���ٶȼ���ͼ
subplot(2, 1, 2);
hold on;

for i = 1:3
    % ������ٶȼ�Allan���� (��g)
    [avar_acc, tau_acc] = allanvar(acc(:,i)*1e6, imu_dt, numPoints);
    
    % �洢Allan��������
    allan_data.accel{i}.avar = avar_acc;
    allan_data.accel{i}.tau = tau_acc;
    
    % ���Ƽ��ٶȼ�Allanƫ��
    loglog(tau_acc, avar_acc, colors{i});
    
    % ���ٶȼ�ƫ�ò��ȶ���
    index = tau_acc < 1e3;
    avar_acc_filtered = avar_acc(index);
    tau_acc_filtered = tau_acc(index);
    [min_avar_acc, min_idx_acc] = min(avar_acc_filtered);
    min_tau_acc = tau_acc_filtered(min_idx_acc);
    
    % �洢���
    acc_bi(i) = min_avar_acc;
    acc_bi_tau(i) = min_tau_acc;
end

title(['Accelerometer Allan Deviation - ', base_name]);
xlabel('Averaging time, �� (s)');
ylabel('Allan Deviation (��g)');
legend(axis_names);
grid on;
set(gca, 'XScale', 'log', 'YScale', 'log');
hold off;

% ����Allan��������info�ṹ��
info.allan_data = allan_data;

%% ��ʾ�������
fprintf('Allan Variance Analysis Results:\n\n');

% �����ǽ��
fprintf('Gyroscope Bias Instability:\n');
for i = 1:3
    fprintf('  %s-axis: %.6f deg/h at %.1f s\n', axis_names{i}, gyr_bi(i), gyr_bi_tau(i));
end
fprintf('\n');

% ���ٶȼƽ��
fprintf('Accelerometer Bias Instability:\n');
for i = 1:3
    fprintf('  %s-axis: %.6f ��g at %.1f s\n', axis_names{i}, acc_bi(i), acc_bi_tau(i));
end
fprintf('\n');

%% GJB 10s�ȶ��� - ʹ���м�1Сʱ����
fprintf('GJB 10s�ȶ���:\n');

% �����м�1Сʱ���ݵ���ʼ�ͽ�������
one_hour_samples = round(1 * 3600 * Fs);
total_available_samples = total_samples;

% ȷ�����㹻������
if total_available_samples < one_hour_samples
    gjb_sample_count = total_available_samples;
    start_idx = 1;
else
    gjb_sample_count = one_hour_samples;
    % �����м�ε���ʼ����
    start_idx = max(1, round((total_available_samples - gjb_sample_count) / 2));
end
end_idx = start_idx + gjb_sample_count - 1;
end_idx = min(end_idx, total_available_samples);

% ��ȡ�м������
acc_gjb = acc(start_idx:end_idx, :);
gyr_gjb = gyr(start_idx:end_idx, :);

fprintf('ʹ�����ݶ�: �� %.2f Сʱ�� %.2f Сʱ (�� %.2f Сʱ)\n',  (start_idx-1)/Fs/3600, end_idx/Fs/3600, (end_idx-start_idx+1)/Fs/3600)

% ������10s�ȶ���
fprintf('Gyroscope:\n');
gyr_std = zeros(3, 1);
for i = 1:3
    % ת��Ϊdeg/h
    gyr_std(i) = gjb_10s_stability(gyr_gjb(:,i), Fs) * 3600;
    fprintf('  %s-axis: %.6f deg/h\n', axis_names{i}, gyr_std(i));
end
fprintf('\n');

% ���ٶȼ�10s�ȶ���
fprintf('Accelerometer:\n');
acc_std = zeros(3, 1);
for i = 1:3
    % ת��Ϊ��g
    acc_std(i) = gjb_10s_stability(acc_gjb(:,i), Fs) * 1e6;
    fprintf('  %s-axis: %.6f ��g\n', axis_names{i}, acc_std(i));
end
fprintf('\n');

%% �������������Ϣ�ṹ��(������Ϊ��������ʱ)
if ~is_script
    % ����Allan���������
    allan_results = table();
    allan_results.FileName = repmat({base_name}, 6, 1);
    allan_results.SensorType = [repmat({'Gyro'}, 3, 1); repmat({'Accel'}, 3, 1)];
    allan_results.Axis = [axis_names'; axis_names'];
    allan_results.BiasInstability = [gyr_bi; acc_bi];
    allan_results.BiasInstabilityTau = [gyr_bi_tau; acc_bi_tau];
    allan_results.Unit = [repmat({'deg/h'}, 3, 1); repmat({'��g'}, 3, 1)];
    
    % ����GJB 10s�ȶ��Խ�����
    gjb_results = table();
    gjb_results.FileName = repmat({base_name}, 6, 1);
    gjb_results.SensorType = [repmat({'Gyro'}, 3, 1); repmat({'Accel'}, 3, 1)];
    gjb_results.Axis = [axis_names'; axis_names'];
    gjb_results.Stability10s = [gyr_std; acc_std];
    gjb_results.Unit = [repmat({'deg/h'}, 3, 1); repmat({'��g'}, 3, 1)];
    
    % ���Allan�����GJB�ȶ��Խ����info�ṹ��
    info.gyro_bias_instability = gyr_bi;
    info.gyro_bias_instability_tau = gyr_bi_tau;
    info.accel_bias_instability = acc_bi;
    info.accel_bias_instability_tau = acc_bi_tau;
    info.gyro_stability_10s = gyr_std;
    info.accel_stability_10s = acc_std;
    
    % ����������Ϣ�ṹ��
    data_info = info;
else
    % �����Ϊ�ű����У�ֻΪ�˱��ֺ���ǩ��һ��
    allan_results = [];
    gjb_results = [];
    data_info = [];
end

end
