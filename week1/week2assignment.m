clearvars;
close all;
%% importing data and stuff
data1 = readtable("Trial1.xlsx");
data1(1:7,:)=[]; % make top 7 rows empty
data1(:,12:14)=[]; % make last few columns empty
data1array = table2array(data1);
Frame = data1(:,1);
Frame = table2array(Frame);
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
plot3(X1,Y1,Z1); hold on;
plot3(X2,Y2,Z2);
plot3(X3,Y3,Z3);
line([X1(1),X2(1),X3(1)],[Y1(1),Y2(1),Y3(1)],[Z1(1),Z2(1),Z3(1)]);hold off;
view([36.6 8.4]);
%% figure 2
figure;
subplot(3,1,1);
plot(Time, X1);
ylabel("hand X position")
xlabel("Time")
subplot(3,1,2);
plot(Time, Y1);
ylabel("hand Y position")
xlabel("Time")
subplot(3,1,3);
plot(Time, Z1);
xlabel("Time")
ylabel("hand Z position")
% diff
%% figure 3
figure
dX1 = diff(X1);
dTime = diff(Time);
xvel = dX1./dTime;
xvel = [xvel; xvel(end)];
subplot(2,1,1)
plot(Time,xvel)
hold on;
xlabel("Time")
ylabel("X Velocity")
% finding peakkkkkk
[peakX, locX] = findpeaks(xvel, Time);
scatter(locX,peakX);
subplot(2,1,2)
plot(Time,X1)
ylabel("X position")
xlabel("Time")
%% finding the phase diagram
figure
plot(X1,xvel);
hold on;
y=[0,0];
x=[0.05,0.15];
plot(x,y)
%% distance between shoulder hand using coords figure 5
figure
shX = X3-X1;
shY = Y3-Y1;
shZ = Z3-Z1;
shXsquared = shX.^2;
shYsquared = shY.^2;
shZsquared = shZ.^2;
sumcoords = sqrt(shXsquared + shYsquared + shZsquared);
plot(Time,sumcoords)
% distance between shoulder hand using norm
dist = [shX, shY, shZ];
d = size(dist);
n = zeros(d(1),1);
for i = 1:d(1)
    n(i,1) = norm(dist(i,:),2);
end
figure 
plot(Time,n)
title('Distance vs Time')
xlabel('Time (sec)')
ylabel('Distance from hand to shoulder')
%% angle over time
% Number of data points
numPoints = length(Frame);

% Initialize angles array
angles = zeros(numPoints, 1);

% angles
for i = 1:numPoints
    % Shoulder to Elbow (stoe) and Elbow to Hand (etoh)
    stoe = [X3(i) - X2(i), Y3(i) - Y2(i), Z3(i) - Z2(i)];
    etoh = [X2(i) - X1(i), Y2(i) - Y1(i), Z2(i) - Z1(i)];

    % Dot product
    dotProduct = dot(stoe, etoh) / (norm(stoe) * norm(etoh));
    dotProduct = max(min(dotProduct, 1), -1);
    angles(i) = rad2deg(acos(dotProduct));
end

% Plotting
plot(Time, angles);
xlabel('Time');
ylabel('Elbow Angle (degrees)');
title('Elbow Angle Over Time');