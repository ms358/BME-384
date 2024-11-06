clc;
clear all;
close all;
load Final_DataSet.mat;
xyz = readtable("Final_DataSet.csv");
xyz(1:6,:) = [];
xyz(:,6:end) = [];
xyz(1202:end,:) = [];
% blanking out useless data, and the last few seconds, which werent synced
% up
xyz = table2array(xyz); % converting table to an array because matlab
time = xyz(:,2);
x = xyz(:,3);
y = xyz(:,4);
z = xyz(:,5);
figure
plot(time,x) % plot x vs time
hold on
plot(time,z) % plot z vs time
legend("xdata", "zdata")
xlabel("time")
% converting time thingies to make them match up
forcetime = linspace(0,10,100);
emgtime = linspace(0,10,10000);
figure
yyaxis left
plot(forcetime,Forcedata)
ylabel("Pinch Force (N), EMG")
hold on
plot(emgtime,EMGdata, "Color","g")
yyaxis right
plot(time,x)
legend("forcedata","emgdata","datax")
title("Pinch Force (N) and EMG, and X (m)")
ylabel("X(m)")
xlabel("time(ms)")
[correlation, lag] = xcorr(EMGdata,Forcedata, 10); 
hold off
figure
stem(lag, correlation)
xlabel("Time Offset (milliseconds)")
ylabel("Cross Correlation")
% The time offset has the highest correlation at around 9 milliseconds.
% animating the point thingy
figure
point = plot3(x(1),y(1),z(1),"o");
for i = 1:length(x)
    set(point, "XData",x(i),"YData",y(i),"ZData",z(i));
    drawnow
    pause(0.0001)
end