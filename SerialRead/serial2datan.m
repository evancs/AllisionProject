function [data]=serial2datan(rawdata)

%% Create data matrix
% reference pg. 99 digiUser Manual
% receive packet is not the same as the transmit packet from sensor nodes

datalength = 3; % must match datapacket length in Arduino
flag = 126; % binary 1111110 Hex 7E

% find all instances of the start byte
flagind=find(rawdata==flag);

% initialize data
data = zeros(length(flagind)-1,datalength);

dataoffset = 14; %data offset from start byte
idoffset = 11; % id offset from start byte

for i = 1:(length(flagind)-1)
    
    idbyte = rawdata(flagind(1)+ idoffset)
    if idbyte ==160
        'A0'
    elseif idbyte == 196
        'C4'
    end
    
    j=1;
    for k = 1:2:(2*datalength-1)
        msb = rawdata(flagind(1)+ dataoffset + k);
        lsb = rawdata(flagind(1)+ dataoffset + k + 1);
        binn=[dec2bin(msb,8) dec2bin(lsb,8)];
        data(i,j) = bin2Q(binn,16);
        j=j+1;
    end
    
end



