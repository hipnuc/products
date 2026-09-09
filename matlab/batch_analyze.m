function summary = batch_analyze(folder)
%BATCH_ANALYZE Analyze the HI91 CSV files directly inside a folder.
% Example: summary = batch_analyze('recordings')
% Write/replace <folder>/analysis_results/summary.csv. Failed files get an error row;
% successful files get six axis rows. No per-file figures are opened.

files = dir(fullfile(folder, '*.csv'));
files = files(~[files.isdir]);
if isempty(files)
    error('hipnuc:NoFiles', 'No CSV files found in %s.', folder);
end
summary = table();
succeeded = 0;
for i = 1:numel(files)
    name = files(i).name;
    fprintf('[%d/%d] %s\n', i, numel(files), name);
    try
        result = analyze_imu(fullfile(folder, name), false);
        row = result.summary;
        row.File = repmat({name}, height(row), 1);
        row.Error = repmat({''}, height(row), 1);
        succeeded = succeeded + 1;
    catch failure
        fprintf(2, '%s: %s\n', name, failure.message);
        row = table({''}, {''}, NaN, NaN, NaN, {name}, {failure.message}, ...
            'VariableNames', {'Axis', 'Unit', 'MinAllanDeviation', 'MinAllanTau_s', ...
            'StdOf10sMovingMean', 'File', 'Error'});
    end
    summary = [summary; row]; %#ok<AGROW>
end
output_folder = fullfile(folder, 'analysis_results');
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end
output_file = fullfile(output_folder, 'summary.csv');
writetable(summary, output_file);
if succeeded == 0
    error('hipnuc:NoValidFiles', 'All %d files failed. Details: %s', numel(files), output_file);
end
fprintf('Analyzed %d/%d files. Summary: %s\n', succeeded, numel(files), output_file);
end
