function [data repeatind flagind]=serial2datan(rawdata,col,flag)

%% Create data matrix
% find startindex
flagind=find(rawdata==flag);
repeatind=find(diff(flagind)==1);
if(isempty(repeatind))
    error('Find:NoFlag','no flag found');
end
startind=flagind(repeatind(1));

% in case wrong double flag
% i=2;
% while(mean(rawdata(startind:2*col:startind*50))~=flag)
%     startind=flagind(repeatind(i));
%     if i==length(repeatind)
%         'too many failed packets'
%         break
%     end
%     i=i+1;
% end


% truncate excess at beginning & end
rawdatatrunc=rawdata(startind:end);
excess=mod(length(rawdatatrunc),col);
if excess~=0
    rawdatatrunc=rawdatatrunc(1:end-excess);
end

% total number of rows
row=length(rawdatatrunc)/(2*col);
data=zeros(row,col);

k=1;
z=1;
m = 1;
while z<length(repeatind);
    k = flagind(repeatind(z));    
    if z>1
        if(flagind(repeatind(z))-flagind(repeatind(z-1))>=2*col)
            for n=1:col;
                %msb=rawdatatrunc(k); % most significant bit
                %lsb=rawdatatrunc(k+1); % least significant bit
                msb = rawdata(k);
                lsb = rawdata(k+1);
                binn=[dec2bin(msb,8) dec2bin(lsb,8)];% 8 allows zeropadding
                data(m,n)=bin2Q(binn,16); % internet version, much faster
                k=k+2; % use everyother ap index
            end
            m=m+1;
        end
    end
    z = z+1;
end
