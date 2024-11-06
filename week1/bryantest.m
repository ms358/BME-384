data=xlsread('Trial1.xlsx');
data1=readtable('Trial1.xlsx');
data1(1:7,:)=[];
data1(:,12:14)=[];
data1array=table2array(data1);
%plot(data1array(:,6),data1array(:,7))
%hold on
%plot(data1array(:,2),data1array(:,3))
%plot(data1array(:,2),data1array(:,6),'Color','g')
%xlabel("Time (ms)")
%%
Frame=data1array(:,1);
Time=data1array(:,2);
X1=data1array(:,3);
Y1=data1array(:,4);
Z1=data1array(:,5);
X2=data1array(:,6);
Y2=data1array(:,7);
Z2=data1array(:,8);
X3=data1array(:,9);
Y3=data1array(:,10);
Z3=data1array(:,11);
plot(Time,X1,'Color','g','LineWidth','3')
hold on
plot(Time,X2,'Color','r','LineWidth','1')