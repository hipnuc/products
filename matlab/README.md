# MATLAB examples

[中文](README_zh.md)

Plot and analyze **single-device HI91 recordings** using MATLAB R2016b or
later: CSV from CHCenter, or SI JSONL from the Python SDK
(`hihost read --record samples.jsonl`). No additional toolboxes are required.

Open this directory in MATLAB and try the small synthetic recording:

```matlab
plot_imu('sample_hi91.csv')
result = analyze_imu('sample_hi91.csv');
```

Replace the filename with your CHCenter CSV or SDK JSONL. Record HI91 output
at a fixed rate and keep the device still for Allan analysis. Record for more
than 10 seconds; short recordings demonstrate
the code but cannot characterize long-term noise.

| Function | Purpose |
| --- | --- |
| `read_hi91_csv(filename)` | Return measurements as a normal MATLAB table. |
| `read_hipnuc_jsonl(filename)` | Return the same measurement columns and units from an SDK HI91 JSONL recording. |
| `plot_imu(filename)` | Plot acceleration, angular velocity, magnetic field and attitude. |
| `analyze_imu(filename)` | Plot Allan deviation and return a numerical summary. Pass `false` as the second argument to suppress figures. |
| `batch_analyze(folder)` | Analyze the CSV and JSONL files in a folder without opening figures; write `<folder>/analysis_results/summary.csv`. |

`plot_imu`, `analyze_imu` and `batch_analyze` select the reader from the file
extension and use the shared measurement columns. CSV may contain additional
columns such as `pc_counter`.

```matlab
data = read_hi91_csv('sample_hi91.csv');
time_s = (data.sys_time - data.sys_time(1)) / 1000;
plot(time_s, data.gyr_x)
xlabel('Time (s)'); ylabel('Angular velocity (deg/s)')

summary = batch_analyze('recordings');
```

CSV columns keep CHCenter's units: `sys_time` in ms, `acc_x/y/z` in G,
`gyr_x/y/z` in deg/s, `mag_x/y/z` in uT, and `roll/pitch/imu_yaw` in degrees.
The JSONL reader converts the SDK's SI fields to those same columns and units.
For these products, acceleration in G converts to m/s² by multiplying by 9.8.
Other message types are skipped; a file without HI91 samples cannot be analyzed.
Repeated HI91 headers are accepted in CSV. Record one device per file.

The reader reports missing headers, invalid numbers and non-increasing time.
Analysis also checks regular sampling, allowing the 1 ms timestamp resolution;
it does not fill gaps or resample. An invalid file is recorded as an error in
the batch summary; if all files fail, batch analysis raises an error. Running
the batch again replaces the generated summary, never the input recordings.

The analysis is an example, not a standards-compliance or product-performance
assessment:

- Allan deviation uses consecutive, non-overlapping cluster means. Averaging
  time is the integer cluster size times the sample period. An incomplete final
  cluster is omitted. The reported minimum is the **minimum Allan deviation**,
  not a fitted bias-instability coefficient.
- The 10-second statistic is the sample standard deviation of full sliding-window
  means. The window is the nearest whole number of samples to 10 seconds; endpoint
  windows containing fewer samples are excluded. It uses the central hour, or the
  whole shorter recording. The result includes the actual interval and window.
- Gyroscope analysis is expressed in deg/h; acceleration analysis in micro-G.
  The supplied CSV is synthetic demonstration data, not a device measurement.

The `.m` files are ordinary functions, ready to read and copy. They do not
clear your workspace, close existing figures, or require a MAT-file conversion.
