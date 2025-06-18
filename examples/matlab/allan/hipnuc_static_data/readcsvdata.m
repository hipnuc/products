clear;
clc;
close all;

% 定义输入文件路径
inputFile = '1xb_icm45686.csv';

% 读取IMU数据
data = readtable(inputFile);

% 提取加速度和陀螺仪数据
acc = [data.acc_x, data.acc_y, data.acc_z];
gyr = [data.gyr_x, data.gyr_y, data.gyr_z];

% 创建带标签的table
imudata = table(data.sys_time, acc(:,1), acc(:,2), acc(:,3), gyr(:,1), gyr(:,2), gyr(:,3), ...
    'VariableNames', {'sys_time', 'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z'});

% 添加Description字段
metadata.description = 'HiPNUC单阵列产品,适用于 HI04M,HIXXM,CH010.';

% 获取输入文件名并保存
[~, filename, ~] = fileparts(inputFile);
outputFile = [filename, '.mat'];
save(outputFile, 'imudata', 'metadata');

% 显示确认信息
fprintf('IMU数据已成功保存为%s文件\n', outputFile);
