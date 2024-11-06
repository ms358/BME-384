clc
clear vars
close all
%% Part 1 Stuff
% This is Bryan's signature with dominant hand (left)
figure(1)
handcoords = readtable("bryan dominant.csv");
handcoords(1:4,:)=[]; % make top 4 rows empty
data=table2array(handcoords);
data = data*100; % converts to centimeters
hold on
thumb = [data(:,9),data(:,10),data(:,11)];
[B, A] = butter(4, 10/60, "low");
thumbxfiltered = filtfilt(B,A,data(:,9));
thumbyfiltered = filtfilt(B,A,data(:,10));
thumbzfiltered = filtfilt(B,A,data(:,11));
thumbfiltered = [thumbxfiltered,thumbyfiltered, thumbzfiltered];
[L,R,K] = curvature(thumbfiltered);
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
plot(L,R)
Time = data(:,1);
Vel = diff(L)./diff(Time);
Vel = [Vel;Vel(end)];
VelX = [diff(thumbxfiltered),diff(thumbyfiltered),diff(thumbzfiltered)]./diff(Time);
VelX = [VelX;VelX(end,:)];
plot(Time,Vel)
legend("pen");
title('dominant hand')
hold off
axis equal