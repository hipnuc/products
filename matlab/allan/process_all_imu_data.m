function process_all_imu_data()
% 批处理脚本，用于分析 hipnuc_static_data 文件夹下的所有 IMU 数据
% 并将结果汇总到一个简洁易读的 CSV 文件中，使用英文表头

% 设置文件夹路径
data_folder = 'hipnuc_static_data';

% 获取所有 .mat 文件
mat_files = dir(fullfile(data_folder, '*.mat'));
num_files = length(mat_files);

if num_files == 0
    error('未在 %s 文件夹中找到任何 .mat 文件', data_folder);
end

fprintf('找到 %d 个 .mat 文件，开始处理...\n', num_files);

% 初始化结果表格
summary_table = table();

% 处理每个文件
for i = 1:num_files
    file_name = mat_files(i).name;
    file_path = fullfile(data_folder, file_name);
    
    fprintf('\n[%d/%d] 正在处理: %s\n', i, num_files, file_name);
    
    try
        % 调用 Allan 分析函数
        [~, ~, data_info] = allan_analysis(file_path);
        
        % 创建一行汇总数据
        row = table();
        
        % 基本信息
        row.FileName = {file_name};
        row.Description = {data_info.description};
        row.SamplingRate_Hz = data_info.sampling_rate;
        row.Duration_Hours = data_info.total_time_hours;
        
        % Allan方差分析 - 陀螺仪零偏稳定性 (deg/h)
        row.Allan_Gyro_Instability_X = data_info.gyro_bias_instability(1);
        row.Allan_Gyro_Instability_Y = data_info.gyro_bias_instability(2);
        row.Allan_Gyro_Instability_Z = data_info.gyro_bias_instability(3);
        
        % Allan方差分析 - 加速度计零偏稳定性 (μg)
        row.Allan_Accel_Instability_X = data_info.accel_bias_instability(1);
        row.Allan_Accel_Instability_Y = data_info.accel_bias_instability(2);
        row.Allan_Accel_Instability_Z = data_info.accel_bias_instability(3);
        
        % GJB 10s 稳定性 - 陀螺仪 (deg/h)
        row.GJB_Gyro_Stability_X = data_info.gyro_stability_10s(1);
        row.GJB_Gyro_Stability_Y = data_info.gyro_stability_10s(2);
        row.GJB_Gyro_Stability_Z = data_info.gyro_stability_10s(3);
        
        % GJB 10s 稳定性 - 加速度计 (μg)
        row.GJB_Accel_Stability_X = data_info.accel_stability_10s(1);
        row.GJB_Accel_Stability_Y = data_info.accel_stability_10s(2);
        row.GJB_Accel_Stability_Z = data_info.accel_stability_10s(3);
        
        % 添加到汇总表格
        summary_table = [summary_table; row];
        
        fprintf('成功处理: %s\n', file_name);
    catch ME
        fprintf('处理文件 %s 时出错: %s\n', file_name, ME.message);
        fprintf('错误详情: %s\n', getReport(ME));
    end
end

% 保存结果
timestamp = datestr(now, 'yyyymmdd_HHMMSS');
result_folder = 'allan_results';

% 如果结果文件夹不存在，则创建
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

% 保存汇总表格
summary_file = fullfile(result_folder, sprintf('IMU_Analysis_Summary_%s.csv', timestamp));
writetable(summary_table, summary_file);
fprintf('\n汇总分析结果已保存至: %s\n', summary_file);

% 显示汇总统计
fprintf('\n===== 汇总统计 =====\n');
fprintf('处理的文件总数: %d\n', num_files);
fprintf('成功处理的文件数: %d\n', height(summary_table));
fprintf('处理失败的文件数: %d\n', num_files - height(summary_table));

% 如果有处理失败的文件，提供更多信息
if num_files > height(summary_table)
    fprintf('\n注意: 部分文件处理失败，请检查上述错误信息\n');
end

end
