function[] = plotdata(D)
%% plot data depending on data channel /100 is to scale the units back, in
% arduino data is multiplied by 100 since it's integer data sent.
data_exp = D.Data;


close all
figure
plot(data_exp(:,2),'b');title(D.Header_Data{2});
figure
plot(data_exp(:,3),'b');title(D.Header_Data{3});
figure
plot(data_exp(:,4),'b');title(D.Header_Data{4});
figure
plot(data_exp(:,5),'b');title(D.Header_Data{5});
figure
plot(data_exp(:,6),'b');title(D.Header_Data{6});
figure
plot(data_exp(:,7),'b');title(D.Header_Data{7});
figure
plot(data_exp(:,8),'b');title(D.Header_Data{8});
figure; hold on
plot(data_exp(:,9),'b');title(D.Header_Data{9});
plot([1 length(data_exp(:,4))],[315 315],'r')
plot([1 length(data_exp(:,4))],[75 75],'r')
figure
plot(data_exp(:,10),'b');title(D.Header_Data{10});