function data = read_hi91_csv(filename)
%READ_HI91_CSV Read a single-device CHCenter HI91 recording into a table.
% Columns retain their CSV names and units: sys_time in ms, acceleration
% in G, angular velocity in deg/s, magnetic field in uT, angles in degrees.
% Other message types are skipped. Repeated HI91 headers are supported.
% The CSV does not identify devices; record one device per file.

fid = fopen(filename, 'rt');
if fid < 0
    error('hipnuc:CannotOpenFile', 'Cannot open CSV file: %s', filename);
end
cleanup = onCleanup(@() fclose(fid));
names = {};
order = [];
rows = [];
count = 0;
line_number = 0;
required = {'sys_time', 'acc_x', 'acc_y', 'acc_z', ...
    'gyr_x', 'gyr_y', 'gyr_z'};

while true
    line = fgetl(fid);
    if ~ischar(line)
        break;
    end
    line_number = line_number + 1;
    % Some CSV editors add a UTF-8 byte order mark to the first line.
    if line_number == 1
        line = strrep(line, char(65279), '');
        if numel(line) >= 3 && isequal(double(line(1:3)), [239 187 191])
            line = line(4:end);
        end
    end
    if ~strncmp(line, 'HI91,', 5)
        continue;
    end
    body = line(6:end);
    % Parse numeric rows directly: splitting every value into a separate
    % string is unnecessarily slow for hours-long recordings.
    [values, parsed, ~, next] = sscanf(body, '%f,');
    if parsed == 0
        fields = strtrim(strsplit(body, ',', 'CollapseDelimiters', false));
    else
        fields = {};
    end
    if any(strcmp(fields, 'sys_time')) || any(strcmp(fields, 'pc_counter'))
        if ~all(cellfun(@isvarname, fields)) || numel(unique(fields)) ~= numel(fields)
            error('hipnuc:InvalidHeader', 'Invalid HI91 header at line %d.', line_number);
        end
        if ~all(ismember(required, fields))
            error('hipnuc:MissingColumns', 'HI91 header at line %d lacks time, acceleration or gyroscope columns.', line_number);
        end
        if isempty(names)
            names = fields;
            rows = zeros(16384, numel(names));
        end
        [found, order] = ismember(names, fields);
        if numel(fields) ~= numel(names) || ~all(found)
            error('hipnuc:ChangedHeader', 'HI91 columns changed at line %d.', line_number);
        end
        continue;
    end
    if isempty(names)
        error('hipnuc:MissingHeader', 'HI91 data precedes its header at line %d.', line_number);
    end
    if parsed ~= numel(names) || sum(body == ',') ~= parsed - 1 || ...
            any(~isfinite(values)) || any(~isspace(body(next:end)))
        error('hipnuc:InvalidRow', 'Invalid HI91 numeric data at line %d.', line_number);
    end
    count = count + 1;
    % Grow geometrically, not once per record; the header sets the column count.
    if count > size(rows, 1)
        rows(2 * size(rows, 1), numel(names)) = 0; %#ok<AGROW>
    end
    rows(count, :) = values(order)'; %#ok<AGROW>
end

if count == 0
    error('hipnuc:NoHi91Data', 'No HI91 measurements found in %s.', filename);
end
data = array2table(rows(1:count, :), 'VariableNames', names);
if any(diff(data.sys_time) <= 0)
    error('hipnuc:InvalidTime', 'HI91 sys_time must increase; check for resets or combined recordings.');
end
end
