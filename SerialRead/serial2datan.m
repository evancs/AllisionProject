function [data]=serial2datan(rawdata)

%% Create data matrix
% reference pg. 99 digiUser Manual
% receive packet is not the same as the transmit packet from sensor nodes
datalength = 3;
flag = 126; % binary 1111110 Hex 7E
% find startindex
flagind=find(rawdata==flag);

data = zeros(length(flagind)-1,datalength);
offset = 14;

for i = 1:(length(flagind)-1)
    
    j=1;
    for k = 1:2:(2*datalength-1)
    msb = rawdata(flagind(1)+ offset+k);
    lsb = rawdata(flagind(1)+ offset+k+1);
    binn=[dec2bin(msb,8) dec2bin(lsb,8)];
    data(i,j) = bin2Q(binn,16);   
    j=j+1;
    end
end



