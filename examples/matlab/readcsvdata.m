% 读取IMU数据
data = readtable('D:\WANGFENG\CHAOHE\CH010-2000\010-1.csv');

% 定义常量
R2D = 180/pi;       % Rad to Deg
D2R = pi/180;       % Deg to Rad
GRAVITY = 9.80665;  % 标准重力加速度 (m/s^2)

% 提取加速度和陀螺仪数据
acc = [data.AccX, data.AccY, data.AccZ];
gyr = [data.GyrX, data.GyrY, data.GyrZ];

% 创建带标签的table
imudata = table(data.TimeStamp, acc(:,1), acc(:,2), acc(:,3), gyr(:,1), gyr(:,2), gyr(:,3), ...
    'VariableNames', {'TimeStamp', 'AccX', 'AccY', 'AccZ', 'GyrX', 'GyrY', 'GyrZ'});

% 添加常量到结构体中
constants.R2D = R2D;
constants.D2R = D2R;
constants.GRAVITY = GRAVITY;

% 保存为.mat文件
save('imudata.mat', 'imudata', 'constants');

% 显示确认信息
disp('IMU数据已成功保存为imudata.mat文件');
