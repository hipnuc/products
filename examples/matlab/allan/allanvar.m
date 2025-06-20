function [sigma, tau] = allanvar(data, dt, numPoints)
    % 增加参数检查
    if nargin < 3
        numPoints = 100;  % 默认值改为100
    end
    
    N = length(data);
    maxT = floor(N/3) * dt;
    minT = 0.01;  % 最小tau就是采样间隔
    
    % 生成更合理的tau范围（1ms到10000s）
    tau = logspace(log10(minT), log10(min(maxT, 10000)), numPoints)';
    
    sigma = zeros(size(tau));
    
    for i = 1:length(tau)
        m = floor(tau(i) / dt);
        m = max(m, 1);
        
        M = floor(N / m) - 1;
        if M < 1
            sigma(i) = NaN;
            continue;
        end
        
        cs = [0; cumsum(data)];
        y = zeros(M+1, 1);
        
        for k = 1:M+1
            start_idx = (k-1)*m + 1;
            end_idx = min(k*m, length(data));
            y(k) = (cs(end_idx+1) - cs(start_idx)) / m;
        end
        
        sigma(i) = sqrt(sum((y(2:end) - y(1:end-1)).^2) / (2*M));
    end
    
    valid = ~isnan(sigma);
    sigma = sigma(valid);
    tau = tau(valid);
end
