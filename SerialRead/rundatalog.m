function [D] = rundatalog(fname)

% Opens serial channel and records xbee data
% converts 2 byte 2's complement binary data to decimal
% forms data matrix where columns are the data of interest
% utilizes bin2Q, much faster 2's complement binary to dec converter
% for serial in general


com = 'COM12'; % specify com channel
% MATCH BAUDRATE WITH XBEE!!!
bdr = 38400;

col_exp = 10; % cole = data columns + 1(for flag)
bfs_exp = 39000; % buffer size for experiment
flag_exp = 252; % binary 11111111
s_exp = serial(com,'BaudRate',bdr,'DataBits',8,'StopBits',1,'InputBufferSize',bfs_exp)
% open serial channel
fopen(s_exp)
fprintf(s_exp,'!!SF1')

% initialize
rawdata_exp = zeros(bfs_exp,1);

bytes = 0;
%keyboard

% Wait for data to become available before reading:
while bytes < 100;
    bytes = s_exp.BytesAvailable;
end

% Read data streaming from XBEE
try
    rawdata_exp = fread(s_exp,bfs_exp);
catch ME
    fclose(s_exp)
    throw(ME)
end

% Close serial object
fclose(s_exp)
delete(s_exp)
clear s_exp

% Convert serial data:
data_exp=serial2datan(rawdata_exp, col_exp, flag_exp);


% Save data:
% data_exp(:,5)=data_exp(:,5)/100; %Accmag*100
% data_exp(:,6)=data_exp(:,6)/100; %gz*100
% data_exp(:,7)=data_exp(:,7)/100; %gy*100
% data_exp(:,9)=data_exp(:,9)/10; % tail_angle*100
% data_exp(:,10)=data_exp(:,10)/100; % thsf*100
% D.Data = data_exp;
% D.Header_Data = {'Flag' 'Time (ms)' 'Pwm Tail (-255 254)' 'Pwm wheel (-255 254)' 'Accmag (g)' 'Acc z (g)' 'Acc Y (g)' 'Gyro (deg/s)' 'Tail Angle (deg)' 'Body Angle Sensor Fusion (deg)'};
% D.RawData = rawdata_exp;

%SaveData(D,fname)

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



end