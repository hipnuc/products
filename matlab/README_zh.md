# MATLAB 示例

[English](README.md)

使用 MATLAB R2016b 或更新版本，读取、绘图和分析 **CHCenter 录制的单设备 HI91 CSV**。
不需要额外工具箱。

在 MATLAB 中打开本目录，先试用附带的小型合成数据：

```matlab
plot_imu('sample_hi91.csv')
result = analyze_imu('sample_hi91.csv');
```

然后将文件名替换为自己的 CHCenter CSV。在 CHCenter 中开启 HI91 输出和 CSV 录制；
做 Allan 分析时使用固定输出率，并保持设备静止。录制时间应超过 10 秒；短数据可以
演示代码，但不能用于判断长期噪声。

| 函数 | 用途 |
| --- | --- |
| `read_hi91_csv(filename)` | 返回普通 MATLAB table，供客户程序直接使用。 |
| `plot_imu(filename)` | 绘制加速度、角速度、磁场和姿态。 |
| `analyze_imu(filename)` | 绘制 Allan 偏差并返回数值汇总；第二个参数传 `false` 可关闭绘图。 |
| `batch_analyze(folder)` | 分析目录中的 CSV，不逐文件开图；写入 `analysis_results/summary.csv`。 |

```matlab
data = read_hi91_csv('sample_hi91.csv');
time_s = (data.sys_time - data.sys_time(1)) / 1000;
plot(time_s, data.gyr_x)
xlabel('Time (s)'); ylabel('Angular velocity (deg/s)')

summary = batch_analyze('recordings');
```

CSV 字段保留 CHCenter 的单位：`sys_time` 为 ms，`acc_x/y/z` 为 G，`gyr_x/y/z`
为 °/s，`mag_x/y/z` 为 μT，`roll/pitch/imu_yaw` 为 °。这些产品的加速度 G 值
乘以 9.8 即为 m/s²。同一文件允许包含其他报文和重复 HI91 表头，但只能录制一台设备：
此 CSV 格式没有设备标识。

读取时会检查缺失表头、异常数值和不递增的时间。分析还会检查采样是否均匀，允许
1 ms 时间戳分辨率造成的量化误差；不补点、不自动重采样。批处理中有问题的文件
会写入汇总的错误行；全部失败时抛出错误。再次批处理会覆盖生成的汇总文件，
不会覆盖输入录制文件。

分析用于演示方法，不宣称符合某项标准或完成产品性能评估：

- Allan 偏差使用连续、不重叠的分组均值。平均时间等于整数分组长度乘以采样周期；
  末尾不足一组的数据不参与计算。输出的最小值是**最小 Allan 偏差**，不是拟合得到的
  零偏不稳定性系数。
- 10 秒统计是完整滑动窗口均值的样本标准差。窗口取最接近 10 秒的整数采样点数，
  排除边缘不足完整窗口的结果。采用中间一小时数据；不足一小时则采用全部数据。
  返回结果中包含实际分析区间和窗口长度。
- 陀螺仪分析使用 °/h，加速度分析使用 micro-G。附带 CSV 为合成演示数据，
  不是真实设备测量结果。

四个 `.m` 文件都是普通函数，可以直接阅读和复制；不会清空工作区、关闭已有图窗，
也不需要先转换为 MAT 文件。
