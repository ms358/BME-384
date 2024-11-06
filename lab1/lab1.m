%% For "Moderate" Condition
clc;
clear variables;
close all;
data1 = readtable("mid.csv");
data1(1:7,:)=[]; 
data1array = table2array(data1);
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
plot3(X1,Y1,Z1);hold on
plot3(X2,Y2,Z2);
plot3(X3,Y3,Z3);
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');
line([X1(1),X2(1),X3(1)],[Y1(1),Y2(1),Y3(1)],[Z1(1),Z2(1),Z3(1)]);
hold off
% Jonathan's weight: 128 lb (58.0598 kg)
% Jonathan's forearm: 0.9289568 kgs
figure;
fframe=table2array(Frame);
numPoints = length(fframe);
angles = zeros(numPoints, 1);
%% angle of elbow over time
r = zeros(size(numPoints));
for i = 1:numPoints
    stoe = [X3(i) - X2(i), Y3(i) - Y2(i), Z3(i) - Z2(i)];
    etoh = [X2(i) - X1(i), Y2(i) - Y1(i), Z2(i) - Z1(i)];
    he = norm(etoh);
    r(i) = he;
    dotProduct = dot(stoe, etoh) / (norm(stoe) * norm(etoh));
    dotProduct = max(min(dotProduct, 1), -1);
    angles(i) = rad2deg(acos(dotProduct));
end
plot(Time, angles);
xlabel('Time');
ylabel('Elbow Angle (degrees)');
title('Elbow Angle Over Time');
%% angular velocity of elbow
figure;
Evel=diff(angles);
Evel=[Evel;Evel(end)];
plot(Time,Evel);
xlabel('Time');
ylabel('Elbow Angular Velocity');
hold on
% finding peaks
[peakX, locX] = findpeaks(Evel, Time,'MinPeakDistance',.2,'MinPeakHeight',1);
scatter(locX,peakX);
%% Angular Acceleration over time
figure;
Eaccel=diff(Evel);
Eaccel=[Eaccel;Eaccel(end)];
plot(Time,Eaccel);
xlabel('Time');
ylabel('Elbow Angular Acceleration');
hold on
% finding peaks
[peakX, locX] = findpeaks(Eaccel, Time,'MinPeakDistance',.4,'MinPeakHeight',0.2);
scatter(locX,peakX);
%%

%% angular velocity vs angular displacement of elbow
figure
plot(angles, Evel)
xlabel('angles displacement');
ylabel('Elbow velocity');
hold on
y = 0.*angles;
plot(angles,y)
%% Finding inertia
m = 58.0598*0.022;
L = 0.28;
RoG = r.*0.827;
intertia = m.*(RoG.^2);
%% Moderate Torque T=Iα
midTorque = intertia.*Eaccel';
midrangeTorque = max(midTorque)-min(midTorque);
midmeanTorque = mean(midTorque);
midstdTorque = std(midTorque); 
disp('For moderate conditions:')
fprintf('Torque Range: %.4f\n', midrangeTorque);
fprintf('Torque Mean: %.4f\n', midmeanTorque);
fprintf('Torque Standard Deviation: %.4f\n', midstdTorque);

%% Moderate Torque Peaks 
midPeakTorque = findpeaks(midTorque);
midrangePeakTorque = max(midPeakTorque)-min(midPeakTorque);
midmeanPeakTorque = mean(midPeakTorque);
midstdPeakTorque = std(midPeakTorque);

fprintf('Peak Torques Range: %.4f\n', midrangePeakTorque);
fprintf('Peak Torques Mean: %.4f\n', midmeanPeakTorque);
fprintf('Peak Torques Standard Deviation: %.4f\n', midstdPeakTorque);




%% Same process but under "As Fast as Possible" Conditions
clear variables
data1 = readtable("nyoooooooooom.csv");
data1(1:7,:)=[]; 
data1array = table2array(data1);
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
figure
plot3(X1,Y1,Z1);hold on
plot3(X2,Y2,Z2);
plot3(X3,Y3,Z3);
xlabel('X (m)');ylabel('Y (m)');zlabel('Z (m)');
line([X1(1),X2(1),X3(1)],[Y1(1),Y2(1),Y3(1)],[Z1(1),Z2(1),Z3(1)]);
hold off
% Jonathan's weight: 128 lb (58.0598 kg)
% Jonathan's forearm: 0.9289568 kgs
figure;
fframe=table2array(Frame);
numPoints = length(fframe);
angles = zeros(numPoints, 1);
%% angle of elbow over time
r = zeros(size(numPoints));
for i = 1:numPoints
    stoe = [X3(i) - X2(i), Y3(i) - Y2(i), Z3(i) - Z2(i)];
    etoh = [X2(i) - X1(i), Y2(i) - Y1(i), Z2(i) - Z1(i)];
    he = norm(etoh);
    r(i) = he;
    dotProduct = dot(stoe, etoh) / (norm(stoe) * norm(etoh));
    dotProduct = max(min(dotProduct, 1), -1);
    angles(i) = rad2deg(acos(dotProduct));
end
angles(268) = 71;
angles(269) = 70;
plot(Time, angles);
xlabel('Time');
ylabel('Elbow Angle (degrees)');
title('Elbow Angle Over Time');
%% angular velocity of elbow
figure;
Evel=diff(angles);
Evel=[Evel;Evel(end)];
plot(Time,Evel);
xlabel('Time');
ylabel('Elbow Angular Velocity');
hold on
% finding peaks
[peakX, locX] = findpeaks(Evel, Time,'MinPeakDistance',.2,'MinPeakHeight',1);
scatter(locX,peakX);
%% Angular Acceleration over time
figure;
Eaccel=diff(Evel);
Eaccel=[Eaccel;Eaccel(end)];
plot(Time,Eaccel);
xlabel('Time');
ylabel('Elbow Angular Acceleration');
hold on
% finding peaks
[peakX, locX] = findpeaks(Eaccel, Time,'MinPeakDistance',.2,'MinPeakHeight',0.2);
scatter(locX,peakX);
%%
%% angular velocity vs angular displacement of elbow
figure
plot(angles, Evel)
xlabel('angles displacement');
ylabel('Elbow velocity');
hold on
y = 0.*angles;
plot(angles,y)
%finding peaks
%% Finding inertia
m = 58.0598*0.022;
L = 0.28;
RoG = r.*0.827;
intertia = m.*(RoG.^2);
%% "As Fast as Possible" Torque T=Iα
fastTorque = intertia.*Eaccel';
fastrangeTorque = max(fastTorque)-min(fastTorque);
fastmeanTorque = mean(fastTorque);
faststdTorque = std(fastTorque); 
disp('For ""As Fast as Possible" conditions:')
fprintf('Torque Range: %.4f\n', fastrangeTorque);
fprintf('Torque Mean: %.4f\n', fastmeanTorque);
fprintf('Torque Standard Deviation: %.4f\n', faststdTorque);

%% "As Fast as Possible" Torque Peaks 
fastPeakTorque = findpeaks(fastTorque);
fastrangePeakTorque = max(fastPeakTorque)-min(fastPeakTorque);
fastmeanPeakTorque = mean(fastPeakTorque);
faststdPeakTorque = std(fastPeakTorque);

fprintf('Peak Torques Range: %.4f\n', fastrangePeakTorque);
fprintf('Peak Torques Mean: %.4f\n', fastmeanPeakTorque);
fprintf('Peak Torques Standard Deviation: %.4f\n', faststdPeakTorque);