function [rawdata_exp] = rundatalog(fname)

% Opens serial channel and records xbee data
% converts 2 byte 2's complement binary data to decimal
% forms data matrix where columns are the data of interest
% utilizes bin2Q, much faster 2's complement binary to dec converter
% for serial in general

% create serial object
com = 'COM21'; % MUST CHECK COM PORT OF XBEE
bdr = 38400; % MATCH BAUDRATE WITH XBEEs!!!
bfs_exp = 22 ; % datapacketsize 
s_exp = serial(com,'BaudRate',bdr,'DataBits',8,'StopBits',1,'InputBufferSize', bfs_exp, 'Parity', 'none');

% open serial channel
fopen(s_exp);

%initialize
rawdata_exp = zeros(bfs_exp+6,10); % datapacketsize + clock size

tic
k=1;
for i = 1:10000
    try
        if s_exp.BytesAvailable==bfs_exp            
            rawdata_exp(1:bfs_exp,k)= fread(s_exp,bfs_exp,'uchar'); 
            rawdata_exp((bfs_exp+1):end,k)=clock;
            k=k+1;
            'hi'
        end
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