function [rawdata_exp] = rundatalog(fname)

% Opens serial channel and records xbee data
% converts 2 byte 2's complement binary data to decimal
% forms data matrix where columns are the data of interest
% utilizes bin2Q, much faster 2's complement binary to dec converter
% for serial in general

% create serial object
com = 'COM21'; % specify com channel
bdr = 38400; % MATCH BAUDRATE WITH XBEE!!!
bfs_exp = 44; % =datapacketsize * number of nodes, i.e. 22 x 2 = 44
s_exp = serial(com,'BaudRate',bdr,'DataBits',8,'StopBits',1,'InputBufferSize',bfs_exp, 'Parity', 'none');

% open serial channel
fopen(s_exp);

%initialize
rawdata_exp = zeros(bfs_exp,10);

tic
% Read data streaming from XBEE
for i = 1:10
    try
        if s_exp.BytesAvailable>0
            datasize = s_exp.BytesAvailable;
            rawdata_exp(1:datasize,i)= fread(s_exp,datasize,'uchar');           
        end
        pause(.5); %pause time set to the same period as Arduino sending rate
    catch ME
        fclose(s_exp)
        throw(ME)
    end    
end
toc

% Close serial object
fclose(s_exp)
delete(s_exp)
clear s_exp

% Convert serial data:
%data_exp = serial2datan(rawdata_exp)

% Save data:
% data_exp(:,5)=data_exp(:,5)/100; %Accmag*100
% data_exp(:,6)=data_exp(:,6)/100; %gz*100
% data_exp(:,7)=data_exp(:,7)/100; %gy*100
% data_exp(:,9)=data_exp(:,9)/10; % tail_angle*100
% data_exp(:,10)=data_exp(:,10)/100; % thsf*100
% D.Data = data_exp;
% D.Header_Data = {'ID' 'Time' 'Data1' 'Data2' 'Data3'};
% D.RawData = rawdata_exp;

%SaveData(D,fname)

end