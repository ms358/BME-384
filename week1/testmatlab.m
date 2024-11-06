%% 1/16/24
% readtable
% import
% importdata
% ls, dir, pwd, clear, close, whos, subplot

% load Trial1.csv
clear all;
% data = xlsread("Trial1.xlsx");
% data(1:7,:)=[]; % make top 7 rows empty
data1 = readtable("Trial1.xlsx");
data1(1:7,:)=[]; % make top 7 rows empty
data1(:,12:14)=[]; % make last few columns empty
data1array = table2array(data1);
%% 1/19/24
% labeling stuff
Frame = data1(:,1);
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
%% 1/16/24
% plot 6 vs 7
data1x = data1array(:,6);
data1y = data1array(:,7);
plot(data1x,data1y)
% plot 2 vs 6
data1x2 = data1array(:,2);
data1y2 = data1array(:,6);
data1y3 = data1array(:,3);
plot(data1x2,data1y2);hold on
plot(data1x2,data1y3)
%% 1/19/24
% plot X1, X2, X1 - X2 vs time
plot(Time,X1,"Color","g","LineWidth",2);hold on
plot(Time,X2,"Color","r","LineWidth",1)
xlabel("Time (seconds)")
ylabel("X(m)")
X1minusX2 = X1-X2;
plot(Time,X1minusX2,"Color","b","LineWidth",1)
save("Trials1Output.m")
%% 1/19/24
% velocity calcs
dX1= diff(X1);
dTime=diff(Time);
velX1=dX1./dTime;
velX1=[velX1;velX1(end)];
plot(Time,velX1);