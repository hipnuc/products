function stability = gjb_10s_stability(data, fs)
% GJS_10S_STABILITY Calculate 10-second stability for a single sensor axis
%
% http://i2nav.cn/index/newListDetail_zw.do?newskind_id=13a8654e060c40c69e5f3d4c13069078&newsinfo_id=43bf2e0076d947aba8b58725ad8f5c15
%
% Inputs:
%   data - Vector of sensor measurements (gyro in deg/s or accel in g)
%   fs   - Sampling frequency in Hz
%
% Outputs:
%   stability - For gyro: stability in deg/h
%               For accel: stability in μg
%               (Caller must apply appropriate unit conversion)
%
% Example:
%   % For gyroscope X-axis
%   gyr_x_stability = gjs_10s_stability(imudata.GyrX, 100) * 3600; % deg/h
%
%   % For accelerometer Y-axis
%   acc_y_stability = gjs_10s_stability(imudata.AccY, 100) * 1e6; % μg

    % Calculate window size for 10-second average
    smooth_time = 10; % 10 second average
    samples_per_smooth = round(smooth_time * fs);
    
    % Apply 10-second moving average
    data_smoothed = smooth(data, samples_per_smooth);
    
    % Calculate standard deviation of smoothed data
    stability = std(data_smoothed);
end
