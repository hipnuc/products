function data = read_hipnuc_jsonl(filename)
%READ_HIPNUC_JSONL Read a single-device HI91 JSONL recording into a table.
% Accepts the SI JSONL written by the Python SDK, for example
% `hihost read --record samples.jsonl`. Columns use the same names and units
% as read_hi91_csv, so the plotting and analysis functions accept either
% source: sys_time in ms, acceleration in G, angular velocity in deg/s,
% magnetic field in uT and angles in degrees. Other message types are
% skipped. JSONL has no device identifier; record one device per file.

fid = fopen(filename, 'rt');
if fid < 0
    error('hipnuc:CannotOpenFile', 'Cannot open JSONL file: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));
names = {};
rows = [];
count = 0;
line_number = 0;

while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end
    line_number = line_number + 1;
    % Some editors add a UTF-8 byte order mark to the first line.
    if line_number == 1
        line = strrep(line, char(65279), '');
    end
    if isempty(strtrim(line))
        continue;
    end
    try
        record = jsondecode(line);
    catch
        error('hipnuc:InvalidRecord', 'Invalid JSON at line %d.', line_number);
    end
    if ~isstruct(record) || ~isscalar(record) || ~isfield(record, 'type') || ...
            ~ischar(record.type) || ~strcmp(record.type, 'HI91')
        continue;
    end
    [values, fields] = hi91_row(record, line_number);
    if isempty(names)
        names = fields;
        rows = zeros(16384, numel(names));
    elseif ~isequal(fields, names)
        error('hipnuc:ChangedFields', 'HI91 fields changed at line %d.', line_number);
    end
    count = count + 1;
    % Grow geometrically, not once per record.
    if count > size(rows, 1)
        rows(2 * size(rows, 1), numel(names)) = 0; %#ok<AGROW>
    end
    rows(count, :) = values;
end

if count == 0
    error('hipnuc:NoHi91Data', 'No HI91 measurements found in %s.', filename);
end
data = array2table(rows(1:count, :), 'VariableNames', names);
if any(diff(data.sys_time) <= 0)
    error('hipnuc:InvalidTime', 'HI91 sys_time must increase; check for resets or combined recordings.');
end
end

function [values, names] = hi91_row(record, line_number)
% SI JSON fields to the CSV column names and units. 1 G is 9.8 m/s^2 here.
map = { ...
    'device_time_ms',         {'sys_time'},                 1; ...
    'acceleration_m_s2',      {'acc_x', 'acc_y', 'acc_z'},  1 / 9.8; ...
    'angular_velocity_rad_s', {'gyr_x', 'gyr_y', 'gyr_z'},  180 / pi; ...
    'magnetic_field_t',       {'mag_x', 'mag_y', 'mag_z'},  1e6; ...
    'euler_rad',              {'roll', 'pitch', 'imu_yaw'}, 180 / pi};
required = 3; % time, acceleration and angular velocity
names = {};
values = [];
for index = 1:size(map, 1)
    field = map{index, 1};
    columns = map{index, 2};
    if ~isfield(record, field) || isempty(record.(field))
        if index <= required
            error('hipnuc:MissingFields', ...
                'HI91 record at line %d lacks %s.', line_number, field);
        end
        continue;
    end
    value = record.(field);
    if ~isnumeric(value) || numel(value) ~= numel(columns) || any(~isfinite(value))
        error('hipnuc:InvalidRecord', 'Invalid %s at line %d.', field, line_number);
    end
    names = [names, columns]; %#ok<AGROW>
    values = [values, map{index, 3} * reshape(double(value), 1, [])]; %#ok<AGROW>
end
end
