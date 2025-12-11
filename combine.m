% Folder with the chunks
folder = 'U:\Chalmers\MCC125\codes\shirzad';

% Pattern for the chunk files
files = dir(fullfile(folder, 'file_*'));

if isempty(files)
    error('No files matching file_* found.');
end

% Extract numeric id from each filename (assumes "file_<id>[.ext]")
ids = zeros(numel(files),1);
for k = 1:numel(files)
    [~, name, ~] = fileparts(files(k).name);     % strip extension
    toks = regexp(name, 'file_(\d+)$', 'tokens', 'once');
    if isempty(toks)
        error('Filename "%s" does not match pattern "file_<number>".', files(k).name);
    end
    ids(k) = str2double(toks{1});
end

[~, order] = sort(ids);
files = files(order);

% Output file
outFile = fullfile(folder, 'test1111111111111111111111.bin');
fout = fopen(outFile, 'w+b');
if fout == -1
    error('Could not open output file "%s".', outFile);
end

cleanupObj = onCleanup(@() fclose(fout));  % ensure close on error

% Concatenate in order
bufSize = 1024*1024; % 1 MB buffer
for k = 1:numel(files)
    inPath = fullfile(folder, files(k).name);
    fin = fopen(inPath, 'rb');
    if fin == -1
        error('Could not open input file "%s".', inPath);
    end
    while true
        data = fread(fin, bufSize, '*uint8');
        if isempty(data)
            break;
        end
        fwrite(fout, data, 'uint8');
    end
    fclose(fin);
end

fprintf('Combined %d files into %s\n', numel(files), outFile);
