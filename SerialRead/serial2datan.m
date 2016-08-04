function [data repeatind flagind]=serial2datan(rawdata,col,flag)

%% Create data matrix
% reference pg. 99 digiUser Manual
% receive packet is not the same as the transmit packet from sensor nodes

% find startindex
flagind=find(rawdata==flag);

for i = 1:(length(flagind)-1)
    msb = rawdata(flagind(1)+ 15);
    lsb = rawdata(flagind(1)+ 16);
    binn=[dec2bin(msb,8) dec2bin(lsb,8)];
    data(i,1) = bin2Q(binn,16);
    
    msb = rawdata(flagind(1)+ 17);
    lsb = rawdata(flagind(1)+ 18);
    binn=[dec2bin(msb,8) dec2bin(lsb,8)];
    data(i,2) = bin2Q(binn,16);
    
    msb = rawdata(flagind(i)+ 19);
    lsb = rawdata(flagind(i)+ 20);
    binn=[dec2bin(msb,8) dec2bin(lsb,8)];
    data(i,3) = bin2Q(binn,16);
end



