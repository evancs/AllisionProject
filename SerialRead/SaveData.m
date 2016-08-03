function SaveData(D,fname)

% If user used pound symbols to automate numbering, get the correct
% filename:
if sum(fname=='#')
    filename = NameFile(pwd,fname);
else
    filename = fname;
end

% Add data to output structure:
S.(filename) = D;

% Save all data:
save(filename,'-struct','S');
sprintf(['Data saved as ' filename '.mat']);

end

function filename = NameFile(SavePath,NamePattern)
% filename = NameFile(SavePath,NamePattern)
% Find the correct name for the file based on a naming pattern and files on
% the save path.  Name patterns should have no spaces or special chars,
% other than '#'.  NameFile will find a block of #s and automatically name
% the file sequentially after the last file on the path.
% Example: NamePattern = 'filename_###'.  Saved filename will be
% 'filename_001' if no similar file exists.  If 'filename_006' exists in
% the SavePath, the next filename will be 'filename_007'.
%
% If the NamePattern does not contain #s, filename = NamePattern.

numslots = (NamePattern=='#');
bufflength = sum(numslots);         % The number of slots for numbers
prefix = NamePattern(~numslots);    % The name pattern without the slots

% Get the filenames in the current path
d = dir(SavePath);
names = char({d.name});

% Look for existing files with prefix:

if size(names,2)<length(prefix)+bufflength+4
    % No file long enough - prefix does not exist
    files = 0;
else
    % Find files that include prefix:
    pfiles = strcmp(cellstr(names(:,1:length(prefix))),prefix);
    
    % Find files that also are the correct length
    lfiles = sum(names~=' ',2)==length(prefix)+bufflength+4;
    
    % Files that are both:
    files = pfiles & lfiles;
end

if sum(files)
    % Get the file numbers by looking in the places they should be:
    nums = str2num(names(files,length(prefix)+1:length(prefix)+bufflength));
    nextnum = sprintf(['%0' num2str(bufflength) '.0f'],max(nums)+1);
    
else
    % There is no such file already saved so start at 1
    nextnum = sprintf(['%0' num2str(bufflength) '.0f'],1);
end

filename = NamePattern;
filename(numslots) = nextnum;

end