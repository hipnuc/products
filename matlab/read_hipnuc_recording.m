function data = read_hipnuc_recording(filename)
%READ_HIPNUC_RECORDING Read a HI91 recording, selected by file extension.
% `.jsonl` uses read_hipnuc_jsonl, anything else read_hi91_csv. Both return
% the same columns and units, so analysis code does not depend on the source.

if ~(ischar(filename) || (isstring(filename) && isscalar(filename)))
    error('hipnuc:InvalidFilename', 'Pass one recording file name.');
end
filename = char(filename);
[~, ~, extension] = fileparts(filename);
if strcmpi(extension, '.jsonl')
    data = read_hipnuc_jsonl(filename);
else
    data = read_hi91_csv(filename);
end
end
