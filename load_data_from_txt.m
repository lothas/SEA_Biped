function data = load_data_from_txt(filename)
%LOAD_DATA_FROM_TXT Summary of this function goes here
%   Detailed explanation goes here

fid = fopen(filename);

% Read all lines & collect in cell array
txt = textscan(fid,'%s','delimiter','\n'); 

% Convert string to numerical value
N = numel(txt{1});
data = cell(N, 1);
for i = 1:N
    vals = strsplit(txt{1}{i},', ');
    if ~isnan(str2double(vals{1}))
        % Value is numeric
        data{i} = str2double(vals);
    else
        % Value is string
        data{i} = vals;
    end
end

end

