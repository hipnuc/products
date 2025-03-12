# IMU Allan方差分析工具

## 简介

本工具用于对IMU（惯性测量单元）静态数据进行Allan方差分析和GJB 10s稳定性分析，可以有效评估IMU的性能指标，包括零偏稳定性、随机游走等关键参数。该工具专为评估高精度IMU设计，支持单文件分析和批量处理功能。

## 主要功能

1. **Allan方差分析**：评估IMU的零偏稳定性（Bias Instability）
2. **国军标10s平滑**：按照GJB标准计算10s平滑
3. **详细数值结果**：提供精确的数值结果，包括零偏稳定性和对应时间常数

## 使用方法

### 基本使用

```
matlab复制% 方法1：使用默认数据文件
allan_analysis

% 方法2：指定数据文件路径
allan_analysis('hipnuc_static_data/example.mat')

% 方法3：获取分析结果作为返回值
[allan_results, gjb_results, data_info] = allan_analysis('hipnuc_static_data/example.mat')
```

### 输入数据格式

输入数据应为MATLAB的.mat文件，包含以下结构：

```
bash复制imudata
  ├── TimeStamp  % 时间戳（毫秒）
  ├── AccX       % X轴加速度（g）
  ├── AccY       % Y轴加速度（g）
  ├── AccZ       % Z轴加速度（g）
  ├── GyrX       % X轴角速度（deg/s）
  ├── GyrY       % Y轴角速度（deg/s）
  └── GyrZ       % Z轴角速度（deg/s）
```

### 输出结果

1. **控制台输出**：
   - 数据集基本信息（采样率、总时长等）
   - Allan方差分析结果（陀螺仪和加速度计的零偏稳定性）
   - 国军标10s稳定性分析结果
2. **图形输出**：
   - 陀螺仪Allan偏差曲线（deg/h）
   - 加速度计Allan偏差曲线（μg）
3. **返回值**（当作为函数调用时）：
   - `allan_results`：Allan方差分析结果表
   - `gjb_results`：GJB 10s稳定性分析结果表
   - `data_info`：数据集信息和分析结果汇总

## 参考标准

- Allan方差分析参考IEEE标准：IEEE-STD-952-1997
- GJB 10s稳定性参考中国军用标准：GJB 2426A-2004

## 注意事项

1. 建议使用至少8小时以上的静态数据，以获得可靠的Allan方差分析结果
2. 数据采集时应确保IMU处于静态状态，避免外部干扰
3. 分析结果中，陀螺仪零偏稳定性单位为deg/h，加速度计零偏稳定性单位为μg
4. 国军标分析使用数据的中间段，以获得最稳定的结果

本工具可帮助工程师全面了解IMU的性能特性，为系统设计和产品选型提供可靠依据。

## 版权信息

© 超核电子 版权所有
 [www.hipnuc.com](http://www.hipnuc.com)