function [rawdata_exp] = rundatalog(fname)

% Opens serial channel and records xbee data
% converts 2 byte 2's complement binary data to decimal
% forms data matrix where columns are the data of interest
% utilizes bin2Q, much faster 2's complement binary to dec converter
% for serial in general

com = 'COM21'; % specify com channel

% MATCH BAUDRATE WITH XBEE!!!
bdr = 38400;
bfs_exp = 50;

% create serial object
s_exp = serial(com,'BaudRate',bdr,'DataBits',8,'StopBits',1,'InputBufferSize',bfs_exp)

% open serial channel
fopen(s_exp)

%initialize
rawdata_exp = zeros(1000,1);
tic
% Read data streaming from XBEE
for i = 1:1000
    try
        rawdata_exp(i) = fread(s_exp,1,'uchar'); % extract 1 byte
    catch ME
        fclose(s_exp)
        throw(ME)
    end
end
toc

tic 
% Close serial object
fclose(s_exp)
delete(s_exp)
clear s_exp
toc

tic
% Convert serial data:
data_exp = serial2datan(rawdata_exp)
toc
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

end