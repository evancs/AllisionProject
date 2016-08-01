function [D] = runtailbot(Inp,fname)

% Opens serial channel and records xbee data
% converts 2 byte 2's complement binary data to decimal
% forms data matrix where columns are the data of interest
% utilizes bin2Q, much faster 2's complement binary to dec converter
% Tom Libby (tlibby@berkeley.edu), Evan Chang-Siu, Jon Beard
% for serial in general

% function_flag = 
        % 1 drop test: automatically turns pwm wheels off, initiates drop
        % hold mode
        % 2 perturbation test, runs with pd control and th_des = 0;
        % 3 jump wall test, switches th_des at specified time
% p_tail = proportional gain rel tail -5
% g_tail = feedforward gain rel tail -45
% p_body = proportional gain absolute body 400
% d_body = derivative gain absolute body 35
% w_pwm_max = maximum wheel pwm value 150
% trial_duration = time in ms for trial length 2500 or 300
% LED_time_start = time in ms for video sync 1275
% th_des_switch = value for th_des after switch, try 45 or 90
% th_des_time_start = time in ms when th_des switches try 1275

%I.trialtype = 'Drop'
%switch strcmp(I.trialtype,'Drop')

%template function 1
% Inp.function_flag  = 1;Inp.p_tail =  -5; Inp.g_tail =  -45; Inp.p_body =  400; Inp.d_body =  23;Inp.w_pwm_max  = 0;Inp.trial_duration = 1000;Inp.LED_time_start = 1000;Inp.th_des_time_start  = 1000;Inp.th_des_switch = 0;

%template function 2
    % passive and no tail
    % Inp.function_flag  = 2;Inp.p_tail =  0; Inp.g_tail =  0; Inp.p_body =  0; Inp.d_body =  0;Inp.w_pwm_max  = 100;Inp.trial_duration = 2500;Inp.LED_time_start = 1250;Inp.th_des_time_start  = 2500;Inp.th_des_switch = 0;
    % pd tail
    % Inp.function_flag  = 2;Inp.p_tail =  0; Inp.g_tail =  0; Inp.p_body =  400; Inp.d_body =  23;Inp.w_pwm_max  = 100;Inp.trial_duration = 2500;Inp.LED_time_start = 1250;Inp.th_des_time_start  = 2500;Inp.th_des_switch = 0;

%template function 3
% Inp.function_flag  = 3;Inp.p_tail =  0; Inp.g_tail =  0; Inp.p_body =  400; Inp.d_body =  23;Inp.w_pwm_max  = 230;Inp.trial_duration = 3000;Inp.LED_time_start = 1175;Inp.th_des_time_start  = 1175;Inp.th_des_switch = 45;


function_flag=Inp.function_flag;
p_tail = Inp.p_tail;
g_tail = Inp.g_tail;
p_body = Inp.p_body;
d_body = Inp.d_body;
w_pwm_max = Inp.w_pwm_max;
trial_duration = Inp.trial_duration;
LED_time_start = Inp.LED_time_start;
th_des_time_start = Inp.th_des_time_start;
th_des_switch = Inp.th_des_switch;

com = 'COM12'; % specify com channel
%bdr = 115200;  % MATCH BAUDRATE WITH XBEE!!!
bdr = 57600;

col_ver = 11; % col = data columns + 1(for flag)
flag_ver = 254; % flag for stream of incoming binary data, 254 is all b11111110, 0xFE, -258 conversion
bfs_ver = col_ver*2; % buffer size for verification
s_ver = serial(com,'BaudRate',bdr,'DataBits',8,'StopBits',1,'InputBufferSize',bfs_ver)
fopen(s_ver)
fprintf(s_ver,['??FF' num2str(function_flag) ',PT' num2str(p_tail) ',GT' num2str(g_tail) ',PB' num2str(p_body) ',DB' num2str(d_body) ',WP' num2str(w_pwm_max) ',TF' num2str(trial_duration) ',LF' num2str(LED_time_start) ',DT' num2str(th_des_time_start) ',DV' num2str(th_des_switch)])  % enter initial parameter vector
rawdata_ver = fread(s_ver,bfs_ver);                % read data back from arduino
fclose(s_ver)
delete(s_ver)
clear s_ver

data_ver=serial2data(rawdata_ver, col_ver, flag_ver);
if(mean(data_ver(end,:) == [-258,function_flag, p_tail, g_tail, p_body, d_body, w_pwm_max, trial_duration, LED_time_start, th_des_time_start, th_des_switch])==1) % verify handshake to proceed
    sprintf('Returned parameters:')
    disp(data_ver(end,:))
    sprintf('Handshake successful, parameters set correctly')
   
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
else
    sprintf('Handshake unsuccessful, exiting')
    fclose(s_ver)
    delete(s_ver)
end

% Save data:
data_exp(:,5)=data_exp(:,5)/100; %Accmag*100
data_exp(:,6)=data_exp(:,6)/100; %gz*100
data_exp(:,7)=data_exp(:,7)/100; %gy*100
data_exp(:,9)=data_exp(:,9)/10; % tail_angle*100
data_exp(:,10)=data_exp(:,10)/100; % thsf*100
D.Data = data_exp;
D.Header_Data = {'Flag' 'Time (ms)' 'Pwm Tail (-255 254)' 'Pwm wheel (-255 254)' 'Accmag (g)' 'Acc z (g)' 'Acc Y (g)' 'Gyro (deg/s)' 'Tail Angle (deg)' 'Body Angle Sensor Fusion (deg)'};
D.RawData = rawdata_exp;
D.Inp = Inp;

SaveData(D,fname)


%% plot data 
plotdata(D);

%% just for acquiring stationary data for TVCF parameters
% sigma1=std(data_exp(2:end,6)/100)
% sigma2=std(data_exp(2:end,7)/100)
% sigma3=std(data_exp(2:end,8)/100)
% sigma4=std(data_exp(2:end,9)/100)
% 
% %sigma1 = sigma1*2;
% %sigma2 = sigma2*5;
% %sigma3 = sigma3*10;
% 
% xo1 = 3*sigma1
% xo2 = 3*sigma2
% xo3 = 3*sigma3
% xo4 = 3*sigma4
% 
% s1 = atan(-0.8)/(-sigma1)
% s2 = atan(-0.8)/(-sigma2)
% s3 = atan(-0.8)/(-sigma3)
% s4 = atan(-0.8)/(-sigma4)


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



%     function y = bin2Q(x,intsize)
%         % y = bin2Q(x,intsize)
%         %
%         % Converts binary data of any format in a string to floating point
%         % decimals of a defined fraction size. the intsize input parameter defines
%         % the size of the exponent and sets the fixed point location in the binary
%         % number. se accompanied matlab program for test of this function
% 
%         B=zeros(length(x),1);
%         for i=1:length(x)
%             if x(i)=='1'
%                 B(i)=1;
%             else
%                 B(i)=0;
%             end
%         end
% 
%         y = 0;
%         k = intsize-1;
%         if B(1)==1
%             sgn=-1;
%         else
%             sgn=0;
%         end
% 
%         y = sgn*2^k;
%         for i = 2:length(B)
%             k=k-1;
%             y=2^k*B(i)+y;
%         end
% 
%     end

end