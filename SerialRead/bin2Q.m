function y = bin2Q(x,intsize)

% bin2Q
% 
% Converts binary data of any format in a string to floating point
% decimals of a defined fraction size. the intsize input parameter defines
% the size of the exponent and sets the fixed point location in the binary
% number. se accompanied matlab program for test of this function

B=zeros(length(x),1);
for i=1:length(x)
    if x(i)=='1' 
        B(i)=1; 
    else 
        B(i)=0;
    end 
end
y = 0;
k = intsize-1;
if B(1)==1 
sgn=-1;
else 
    sgn=0;
end
y = sgn*2^k;
for i = 2:length(B)
     k=k-1;
    y=2^k*B(i)+y;
end
