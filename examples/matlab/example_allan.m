clear;
clc;
close all;

%% ��ȡ����
load('imudata.mat');
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.80665;  % ��׼�������ٶ� (m/s^2)

acc = [imudata.AccX, imudata.AccY, imudata.AccZ];
gyr = [imudata.GyrX, imudata.GyrY, imudata.GyrZ];

% ���㲢��ӡ������Ϣ
total_samples = length(acc);
total_time = (imudata.TimeStamp(end) - imudata.TimeStamp(1)) / 1000; % ��ʱ�䣨�룩
imu_dt = mean(diff(imudata.TimeStamp)) / 1000;
Fs = 1 / imu_dt; % ����Ƶ��

%% ������Ϣ
fprintf('������Ϣ:\n');
fprintf('  ��������: %d\n', total_samples);
fprintf('  ����Ƶ��: %.2f Hz\n', Fs);
fprintf('  �ܲ���ʱ��: %.2f �� (%.2f h)\n', total_time, total_time/3600);
fprintf('  ƽ���������: %.6f ��\n\n', imu_dt);

axis_names = {'X', 'Y', 'Z'};

%% ���ݷ�Χ��Ϣ
fprintf('���ݷ�Χ:\n');
fprintf('  ���ٶȼ� (G):\n');
for i = 1:3
    fprintf('    %s��: [%.6f, %.6f]\n', axis_names{i}, min(acc(:,i)), max(acc(:,i)));
end
fprintf('  ������ (deg/s):\n');
for i = 1:3
    fprintf('    %s��: [%.6f, %.6f]\n', axis_names{i}, min(gyr(:,i)), max(gyr(:,i)));
end
fprintf('\n');

% ����tauֵ��ʹ���������Ĳ������ڣ�
m = logspace(0, log10(floor(length(gyr)/2)), 1000);
m = unique(round(m));  % ȷ��m��Ψһ������ֵ
tau = m / 100;

% ����ͼ��
figure('Name', 'Gyroscope Allan Deviation');
figure('Name', 'Accelerometer Allan Deviation');
colors = { 'r', 'g', 'b'};
for i = 1:3
    % ������Allan�������
    [avar_gyr, tau_gyr] = allanVar(gyr(:,i)*3600, 1/Fs);
    % ���ٶȼ�Allan�������
    [avar_acc, tau_acc] = allanVar(acc(:,i)*1e6, 1/Fs);
    % ����������Allanƫ������
    figure(1);
    loglog(tau_gyr, avar_gyr, colors{i}, 'LineWidth', 2);
    hold on;

    % ���Ƽ��ٶȼ�Allanƫ������
    figure(2);
    loglog(tau_acc, avar_acc, colors{i}, 'LineWidth', 2);
    hold on;

    % ����һЩ�ؼ�����
    % ������
    % �ҵ� tau_gyr С�� 1000 ������
    index = tau_gyr < 1e3;

    % ʹ����Щ�����ҵ���Ӧ�� avar_gyr ֵ
    avar_gyr_filtered = avar_gyr(index);
    tau_gyr_filtered = tau_gyr(index);

    % ���ҹ��˺�� avar_gyr ����Сֵ
    [min_avar, min_index] = min(avar_gyr_filtered);
    min_tau = tau_gyr_filtered(min_index);
    
    B_gyr = min_avar;   % ��ƫ���ȶ��� (deg/s)

    % ���ٶȼ�
    % ʹ����Щ�����ҵ���Ӧ�� avar_gyr ֵ
    avar_acc_filtered = avar_acc(index);
    tau_acc_filtered = tau_acc(index);

    % ���ҹ��˺�� avar_gyr ����Сֵ
    [min_avar, min_index] = min(avar_acc_filtered);

    B_acc = min_avar;   % ��ƫ���ȶ��� (G)

    % ������
    fprintf('Results for %s Axis:\n', axis_names{i});
    fprintf('Gyroscope (Allan Variance):\n');
    fprintf('  ��ƫ���ȶ���: %.6f deg/h\n', B_gyr);
    fprintf('Accelerometer (Allan Variance):\n');
    fprintf('  ��ƫ���ȶ���: %.6f uG\n', B_acc );
end
axis_temp = {'1', '2', '3'};
figure(1);
%title('Root Allan Variance of gyro');
title('Gyroscope Allan Deviation - All Axex');
xlabel('Averaging time,�� (s)');
ylabel('Allan Deviation (deg/h)');
legend(axis_names);
grid on;
hold off;

figure(2);
title('Accelerometer Allan Deviation - All Axes');
xlabel('Averaging time,�� (s)');
ylabel('Allan Deviation (ug)');
legend(axis_names);
grid on;
hold off;

%% ������10sƽ������
% http://i2nav.cn/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=43bf2e0076d947aba8b58725ad8f5c15
% ���������ֵ���Թߵ���ʵ�ʱ����бȽ�ֱ�ӵ�Ӱ�죬����ʵָ�����壻
smooth_time = 10; % 10��ƽ��
samples_per_smooth = round(smooth_time * Fs);

fprintf('������10sƽ���������:\n');
for i = 1:3
    % ������10sƽ��
    gyr_smoothed = smooth(gyr(:,i), samples_per_smooth);

    % ���ٶȼ�10sƽ��
    acc_smoothed = smooth(acc(:,i), samples_per_smooth);

    % ����ƽ����ı�׼��
    gyr_std = std(gyr_smoothed);
    acc_std = std(acc_smoothed);

    % ������
    fprintf('%s��:\n', axis_names{i});
    fprintf('  ��������ƫ�ȶ���: %.6f deg/h\n', gyr_std * 3600);
    fprintf('  ���ٶȼ���ƫ�ȶ���: %.6f uG\n\n', acc_std * 1e6);
end
%Allan������㺯��-psins 
function [sigma, tau, Err] = allanVar(y0, tau0)
    N = length(y0);
    y = y0; NL = N;
    for k = 1:log2(N)
        sigma(k,1) = sqrt(1/(2*(NL-1))*sum((y(2:NL)-y(1:NL-1)).^2)); % diff&std
        tau(k,1) = 2^(k-1)*tau0;      % correlated time
        Err(k,1) = 1/sqrt(2*(NL-1));  % error boundary
        NL = floor(NL/2);
        if NL<3
            break;
        end
        y = 1/2*(y(1:2:2*NL) + y(2:2:2*NL));  % mean & half data length
    end
end