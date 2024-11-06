clc;
clear vars;
close all;
A=open('lab6data1/lab6part1aMatt.csv');
B=readtable('lab6data1/lab61bMatt.csv');
C=readtable('lab6data1/lab62cMatt.csv');
D=readtable('lab6data1/lab62drightMatt.csv');
 
% A(1:7,:)=[]; A(:,15:22)=[];
% B(1:7,:)=[]; B(:,15:22)=[];
% C(1:7,:)=[]; C(:,18:22)=[];
% D(1:7,:)=[]; D(:,18:22)=[];
A=table2array(A);
B=table2array(B);
C=table2array(C);
D=table2array(D);
TimeA=A(:,2);TimeB=B(:,2);TimeC=C(:,2);TimeD=D(:,2);
 
%% plot the 3D trajectories
% Assign the data to a variable for Experiment 1 A (Eyes Open, Wide Target)
 
X1=A(:,3); Y1=A(:,4); Z1=A(:,5);
X3=A(:,9);Y3=A(:,10);Z3=A(:,11);
X4=A(:,12);Y4=A(:,13);Z4=A(:,14);
X2=A(:,6);Y2=A(:,7);Z2=A(:,8);
 
% plot 3-D trajectories of the 3 markers for Experiment 1 A (Eyes Open,
% Wide Target)
 
figure(1);
plot3(X1,Y1,Z1,'b'); 
hold on
plot3(X3,Y3,Z3,'k');
plot3(X4,Y4,Z4,'g');
plot3(X2,Y2,Z2,'r');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Eyes Open, Wide Target');
 
% create a stick figure of the arm. connect shoulder, elbow, and hand with a
% line at time=0
% stick figure for 1 A
line([X1(1) X3(1) X4(1) X2(1)], [Y1(1) Y3(1) Y4(1) Y2(1)], [Z1(1) Z3(1) Z4(1) Z2(1)]);
 
% Animate the stick figure for 1 A
for i=1:30:length(X2)
    view([0 0 1]);
    h1 = line([X1(i) X3(i) X4(i) X2(i)], [Y1(i) Y3(i) Y4(i) Y2(i)], [Z1(i) Z3(i) Z4(i) Z2(i)]);
    h2 = plot3(X2(i,1), Y2(i,1), Z2(i,1), 'o','MarkerSize',10,'MarkerFaceColor','m');
    drawnow
    pause(0.1)
    delete(h1)
    delete(h2)
end
 
% Assign the data to a variable for Experiment 1 B (Eyes Open, Narrow Target)
 
XB1=B(:,3); YB1=B(:,4); ZB1=B(:,5);
XB4=B(:,12);YB4=B(:,13);ZB4=B(:,14);
XB2=B(:,6);YB2=B(:,7);ZB2=B(:,8);
XB3=B(:,9);YB3=B(:,10);ZB3=B(:,11);
 
% plot 3-D trajectories of the 3 markers for Experiment 1 B (Eyes Open,
% Narrow Target)
 
figure(2);
plot3(XB1,YB1,ZB1,'b'); 
hold on
plot3(XB4,YB4,ZB4,'k');
plot3(XB2,YB2,ZB2,'g');
plot3(XB3,YB3,ZB3,'r');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Eyes Open, Narrow Target');
 
% create a stick figure of the arm. connect shoulder, elbow, and hand with a
% line at time=0
% stick figure for 1 B
line([XB1(1) XB4(1) XB2(1) XB3(1)], [YB1(1) YB4(1) YB2(1) YB3(1)], [ZB1(1) ZB4(1) ZB2(1) ZB3(1)]);
 
% Animate the stick figure for 1 B
for i=1:30:length(XB3)
    view([0 0 1]);
    h1 = line([XB1(i) XB4(i) XB2(i) XB3(i)], [YB1(i) YB4(i) YB2(i) YB3(i)], [ZB1(i) ZB4(i) ZB2(i) ZB3(i)]);
    h2 = plot3(XB3(i,1), YB3(i,1), ZB3(i,1), 'o','MarkerSize',10,'MarkerFaceColor','m');
    drawnow
    pause(0.1)
    delete(h1)
    delete(h2)
end
 
% Assign the data to a variable for Experiment 2 C (Eyes Open)
 
XC1=C(:,3); YC1=C(:,4); ZC1=C(:,5);
XC5=C(:,15);YC5=C(:,16);ZC5=C(:,17);
XC4=C(:,12);YC4=C(:,13);ZC4=C(:,14);
XC3=C(:,9);YC3=C(:,10);ZC3=C(:,11);
XC2=C(:,6);YC2=C(:,7);ZC2=C(:,8);
 
% plot 3-D trajectories of the 3 markers for Experiment 2 C (Eyes Open)
 
figure(3);
plot3(XC1,YC1,ZC1,'b'); 
hold on
plot3(XC5,YC5,ZC5,'k');
plot3(XC3,YC3,ZC3,'g');
plot3(XC2,YC2,ZC2,'r');
plot3(XC4,YC4,ZC4,'y');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Experiment 2 C (Eyes Open)');
 
% create a stick figure of the arm. connect shoulder, elbow, and hand with a
% line at time=0
% stick figure for 2 C
line([XC3(1) XC1(1) XC5(1) XC2(1)], [YC3(1) YC1(1) YC5(1) YC2(1)], [ZC3(1) ZC1(1) ZC5(1) ZC2(1)]);
 
% Animate the stick figure for 2 C
for i=1:30:length(XC2)
    view([0 0 1]);
    h1 = line([XC3(i) XC1(i) XC5(i) XC2(i)], [YC3(i) YC1(i) YC5(i) YC2(i)], [ZC3(i) ZC1(i) ZC5(i) ZC2(i)]);
    h2 = plot3(XC2(i,1), YC2(i,1), ZC2(i,1), 'o','MarkerSize',10,'MarkerFaceColor','m');
    drawnow
    pause(0.1)
    delete(h1)
    delete(h2)
end
 
% Assign the data to a variable for Experiment 2 D (Eyes Closed)
 
XD3=D(:,9); YD3=D(:,10); ZD3=D(:,11);
XD1=D(:,3);YD1=D(:,4);ZD1=D(:,5);
XD5=D(:,15);YD5=D(:,16);ZD5=D(:,17);
XD2=D(:,6);YD2=D(:,7);ZD2=D(:,8);
XD4=D(:,12);YD4=D(:,13);ZD4=D(:,14);
 
% plot 3-D trajectories of the 3 markers for Experiment 2 D (Eyes Open)
 
figure(4);
plot3(XD1,YD1,ZD1,'b'); 
hold on
plot3(XD5,YD5,ZD5,'k');
plot3(XD3,YD3,ZD3,'g');
plot3(XD2,YD2,ZD2,'r');
plot3(XD4,YD4,ZD4,'y');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Experiment 2 D (Eyes Closed)');
 
% create a stick figure of the arm. connect shoulder, elbow, and hand with a
% line at time=0
% stick figure for 2 D
line([XD3(1) XD1(1) XD5(1) XD2(1)], [YD3(1) YD1(1) YD5(1) YD2(1)], [ZD3(1) ZD1(1) ZD5(1) ZD2(1)]);
 
% Animate the stick figure for 2 D
for i=1:40:length(XD2)
    view([0 0 1]);
    h1 = line([XD3(i) XD1(i) XD5(i) XD2(i)], [YD3(i) YD1(i) YD5(i) YD2(i)], [ZD3(i) ZD1(i) ZD5(i) ZD2(i)]);
    h2 = plot3(XD2(i,1), YD2(i,1), ZD2(i,1), 'o','MarkerSize',10,'MarkerFaceColor','m');
    drawnow
    pause(0.1)
    delete(h1)
    delete(h2)
end
 
%% plot finger trajectories for the 8 movements 
 
% plot for experiment A
figure(5)
subplot(2,1,1)
plot(TimeA,X2,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 1 AX (Wide Target)');
xlabel('Time(s)');
ylabel('Finger Movements X-Direction');
 
figure(5)
subplot(2,1,2)
plot(TimeA,Z2,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 1 AZ (Wide Target)');
xlabel('Time(s)');
ylabel('Finger Movements Z-Direction');
 
figure(6)
subplot(2,1,1)
plot(TimeB, XB3,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 1 B (Narrow Target)');
xlabel('Time(s)');
ylabel('Finger Movements X-Direction');
 
subplot(2,1,2)
plot(TimeB, ZB3,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 1 B (Narrow Target)');
xlabel('Time(s)');
ylabel('Finger Movements Z-Direction');
 
figure(7)
subplot(2,1,1)
plot(TimeC, XC2,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 2 C (Eyes Open)');
xlabel('Time(s)');
ylabel('Finger Movements X-Direction');
 
subplot(2,1,2)
plot(TimeC,ZC2,'r','LineWidth',3)
title('Trajectories of the Target with Eyes Open');
xlabel('Time(s)');
ylabel('Target Movements Z-Direction');
 
figure(8)
subplot(2,1,1)
plot(TimeD,XD2,'b','LineWidth',3)
title('Trajectories of the Finger for Experiment 2 D (Eyes Closed)');
xlabel('Time(s)');
ylabel('Finger Movements X-Direction');
 
subplot(2,1,2)
plot(TimeD,ZD2,'r','LineWidth',3)
title('Trajectories of the Target with Eyes Closed');
xlabel('Time(s)');
ylabel('Target Movements Z-Direction');
 
%% Using findpeaks function to find the endpoint of the location at the time of movement reversal
figure(21)
plot(TimeC,XC2,'r','LineWidth',3)
hold on
[Xpeaks,Xlocs] = findpeaks(XC2,'MinPeakHeight',0.5);
plot(TimeC(Xlocs),-Xpeaks, 'o'); 
 
figure(22)
plot(TimeC,XC4,'r','LineWidth',3)
hold on
[Xpeaks,Xlocs] = findpeaks(-XC4,'MinPeakHeight', -0.07);
plot(TimeC(Xlocs),-Xpeaks, 'o'); 
 
figure(23)
plot(TimeD,XD2,'b','LineWidth',3)
hold on
[Xpeaks,Xlocs] = findpeaks(-XD2,'MinPeakHeight', 0.1);
plot(TimeD(Xlocs),-Xpeaks, 'o'); 
 
figure(24)
plot(TimeD,XD4,'r','LineWidth',3)
hold on
[Xpeaks,Xlocs] = findpeaks(-XD4,'MinPeakHeight', -0.06);
plot(TimeD(Xlocs),-Xpeaks, 'o'); 
 
%% Scater plot of the targets and the movement endpoints
 
% Eyes open
TrialN = [1 2 3 4 5 6 7 8];
FingerX_2C = [-0.207205 -0.195499 -0.195389 -0.200328 -0.2216 -0.203728 -0.2393 -0.21866];
FingerY_2C = [-0.1649 -0.1745 -0.14337 -0.1775 -0.1620 -0.1478 -0.1761 -0.1718];
FingerZ_2C = [0.8666 0.8142 0.9515 0.8130 0.9091 0.9444 0.8176 0.8337];
 
TargetX_2C = [-0.1979 -0.1864 -0.1501 -0.1849 -0.2887 -0.1801 -0.2193 -0.2060];
TargetY_2C = [-0.1727 -0.1775 -0.1493 -0.1815 -0.1578 -0.1460 -0.1797 -0.1747];
TargetZ_2C = [0.885 0.8467 0.9509 0.8315 0.9304 0.9811 0.8332 0.9304];
 
figure(9)
scatter(TrialN,FingerX_2C,'b*','LineWidth',3)
hold on
scatter(TrialN,FingerY_2C,'ro','LineWidth',3)
scatter(TrialN,FingerZ_2C,'g^','LineWidth',3)
scatter(TrialN,TargetX_2C,'cx','LineWidth',3)
scatter(TrialN,TargetY_2C,'m+','LineWidth',3)
scatter(TrialN,TargetZ_2C,'kd','LineWidth',3)
hold off
title('Scatter Plot for X,Y,Z Coordinate between Target and Finger (Eyes Open)');
xlabel('Trial Numbers')
ylabel('Distance (mm)')
legend('Finger x','Finger y','Finger z','Target x','Target y', 'Target z');
 
TrialN = [1 2 3 4 5 6 7 8];
FingerX_2D = [-0.159331 -0.1809 -0.152983 -0.144481 -0.160952 -0.101232 -0.170463 -0.15974];
FingerY_2D = [-0.1605 -0.1807 -0.1621 -0.1421 -0.1595 -0.1603 -0.1790 -0.1511];
FingerZ_2D = [0.8790 0.8021 0.8889 0.9798 0.8696 0.89768 0.8106 0.9500];
 
TargetX_2D = [-0.164013 -0.149969 -0.141962 -0.141032 -0.165203 -0.1397 -0.157543 -0.168222];
TargetY_2D = [0.1218 0.1093 0.1138 -0.1399 -0.1673 -0.1811 -0.1479 0.0978];
TargetZ_2D = [0.8531 0.7500 0.7702 0.9894 0.8777 0.8161 0.9523 0.8396];
 
figure(10)
scatter(TrialN,FingerX_2D,'b*','LineWidth',3)
hold on
scatter(TrialN,FingerY_2D,'ro','LineWidth',3)
scatter(TrialN,FingerZ_2D,'g^','LineWidth',3)
scatter(TrialN,TargetX_2D,'cx','LineWidth',3)
scatter(TrialN,TargetY_2D,'m+','LineWidth',3)
scatter(TrialN,TargetZ_2D,'kd','LineWidth',3)
hold off
title('Scatter Plot for X,Y,Z Coordinate between Target and Finger (Eyes Closed)');
xlabel('Trial Numbers')
ylabel('Distance (mm)')
legend('Finger x','Finger y','Finger z','Target x','Target y', 'Target z');
 
%% Calculate the distance between the target and the movement reversals
 
% calculate the distance between hand and shoulder an plot it over time
 
% distance for eyes open
DistanceFingerTarget=[];
for i=1:length(XC2)
    E=[XC4(i)-XC2(i), YC4(i)-YC2(i), ZC4(i)-ZC2(i)];
    DistD=norm(E);
    DistanceFingerTarget=[DistanceFingerTarget;DistD];
end
 
figure(11)
plot(TimeC,DistanceFingerTarget,'b','LineWidth',3)
title('Distance between the target and Movement (Eyes Open)')
xlabel('Time(s)')
ylabel('Distance(mm)')
 
% distance for eyes closed
DistanceFingerTarget2=[];
for i=1:length(XD2)
    E=[XD4(i)-XD2(i), YD4(i)-YD2(i), ZD4(i)-ZD2(i)];
    DistD=norm(E);
    DistanceFingerTarget2=[DistanceFingerTarget2;DistD];
end
 
figure(12)
plot(TimeD,DistanceFingerTarget2,'r','LineWidth',3)
title('Distance between the target and Movement (Eyes Closed)')
xlabel('Time(s)')
ylabel('Distance(mm)')
 
%% Find the mean and standard deviation of the distance for each condition
 
MeanDistance_EyesOpen = mean(DistanceFingerTarget);
StandDev_EyesOpen = std(DistanceFingerTarget);
MeanDistance_EyesClosed = mean(DistanceFingerTarget2);
StandDev_EyesClosed = std(DistanceFingerTarget2);
 
%% plot the elbow joint angle vs. time for one trial in each condition
 
% Wide Target
AA=[X3-X4,Y3-Y4,Z3-Z4];
AB=[X1-X3,Y1-Y3,Z1-Z3];
ElbowAngleWideTarget = [];
for i = 1:length(X4)
    AA=[X3(i)-X4(i),Y3(i)-Y4(i),Z3(i)-Z4(i)];
    AB=[X1(i)-X3(i),Y1(i)-Y3(i),Z1(i)-Z3(i)];
    ThetaWideTarget = acos(dot(AA,AB)./(norm(AA)*norm(AB)));
    ElbowAngleWideTarget = [ElbowAngleWideTarget;ThetaWideTarget];
end
 
figure(13);
plot(TimeA,ElbowAngleWideTarget);
xlabel('Time (Sec)');
ylabel('Elbow Angle (Theta)');
title('Elbow Angle for Wide Target');
 
% Narrow Target
BB=[XB4-XB2,YB4-YB2,ZB4-ZB2];
BA=[XB1-XB4,YB1-YB4,ZB1-ZB4];
ElbowAngleNarrowTarget = [];
for i = 1:length(XB2)
    BB=[XB4(i)-XB2(i),YB4(i)-YB2(i),ZB4(i)-ZB2(i)];
    BA=[XB1(i)-XB4(i),YB1(i)-YB4(i),ZB1(i)-ZB4(i)];
    ThetaNarrowTarget = acos(dot(BB,BA)./(norm(BB)*norm(BA)));
    ElbowAngleNarrowTarget = [ElbowAngleNarrowTarget;ThetaNarrowTarget];
end
 
figure(14);
plot(TimeB,ElbowAngleNarrowTarget);
xlabel('Time (Sec)');
ylabel('Elbow Angle (Theta)');
title('Elbow Angle for Narrow Target');
 
% Eyes Open
C1=[XC1-XC5,YC1-YC5,ZC1-ZC5];
C2=[XC3-XC1,YC3-YC1,ZC3-ZC1];
ElbowAngleEyesOpen = [];
for i = 1:length(XC5)
    C1=[XC1(i)-XC5(i),YC1(i)-YC5(i),ZC1(i)-ZC5(i)];
    C2=[XC3(i)-XC1(i),YC3(i)-YC1(i),ZC3(i)-ZC1(i)];
    ThetaEyesOpen = acos(dot(C1,C2)./(norm(C1)*norm(C2)));
    ElbowAngleEyesOpen = [ElbowAngleEyesOpen;ThetaEyesOpen];
end
 
figure(15);
plot(TimeC,ElbowAngleEyesOpen);
xlabel('Time (Sec)');
ylabel('Elbow Angle (Theta)');
title('Elbow Angle for Eyes Open');
 
% Eyes Closed
D1=[XD1-XD5,YD1-YD5,ZD1-ZD5];
D2=[XD3-XD1,YD3-YD1,ZD3-ZD1];
ElbowAngleEyesClosed = [];
for i = 1:length(XD5)
    D1=[XD1(i)-XD5(i),YD1(i)-YD5(i),ZD1(i)-ZD5(i)];
    D2=[XD3(i)-XD1(i),YD3(i)-YD1(i),ZD3(i)-ZD1(i)];
    ThetaEyesClosed = acos(dot(D1,D2)./(norm(D1)*norm(D2)));
    ElbowAngleEyesClosed = [ElbowAngleEyesClosed;ThetaEyesClosed];
end
 
figure(16);
plot(TimeD,ElbowAngleEyesClosed);
xlabel('Time (Sec)');
ylabel('Elbow Angle (Theta)');
title('Elbow Angle for Eyes Closed');
 
%% Calculate and plot the finger velocity for each trial, one plot per condition
 
% Wide Target
xfinger1Adot=diff(X2)./diff(TimeA);
xfinger1Adot=cat(1,xfinger1Adot,xfinger1Adot(end));
 
figure(17)
plot(TimeA,xfinger1Adot,'LineWidth',3)
title('Finger Velocity for Wide Target')
xlabel('Time(s)')
ylabel('Velocity (m/s)')
 
% Narrow Target
xfinger1Bdot=diff(XB3)./diff(TimeB);
xfinger1Bdot=cat(1,xfinger1Bdot,xfinger1Bdot(end));
 
figure(18)
plot(TimeB,xfinger1Bdot,'LineWidth',3)
title('Finger Velocity for Narrow Target')
xlabel('Time(s)')
ylabel('Velocity (m/s)')
 
% Eyes Open
xfinger2Cdot=diff(XC2)./diff(TimeC);
xfinger2Cdot=cat(1,xfinger2Cdot,xfinger2Cdot(end));
 
figure(19)
plot(TimeC,xfinger2Cdot,'LineWidth',3)
title('Finger Velocity for Eyes Open')
xlabel('Time(s)')
ylabel('Velocity (m/s)')
 
% Eyes Closed
xfinger2Ddot=diff(XD2)./diff(TimeD);
xfinger2Ddot=cat(1,xfinger2Ddot,xfinger2Ddot(end));
 
figure(20)
plot(TimeD,xfinger2Ddot,'LineWidth',3)
title('Finger Velocity for Eyes Closed')
xlabel('Time(s)')
ylabel('Velocity (m/s)')
 
%% For each of the four conditions, find the mean and standard deviation of the peak velocity of reaching
 
[peaks,locx] = findpeaks(xfinger1Adot,TimeA);
plot(locx,peaks, 'og', 'MarkerSize',3, 'MarkerFaceColor','m')
 
% Wide Target
MeanWideTarget_FingerVelocity = mean(xfinger1Adot);
StandDev_FV1 = std(peaks)
[peaks,locx] = findpeaks(xfinger1Adot,TimeA);
 
% Narrow Target
MeanNarrowTarget_FingerVelocity = mean(xfinger1Bdot)
StandDev_FV2 = std(peaks)
[peaks,locx] = findpeaks(xfinger1Bdot,TimeB)
 
% Eyes Open
[peaks,locx] = findpeaks(xfinger2Cdot,TimeC);
MeanEyesOpen_FingerVelocity = mean(xfinger2Cdot)
StandDev_FV3 = std(peaks)
 
% Eyes Closed
[peaks,locx] = findpeaks(xfinger2Ddot,TimeD);
MeanEyesClosed_FingerVelocity = mean(xfinger2Ddot)
StandDev_FV4 = std(peaks)
 
%% Using a subplot function create a figure showing the EMG of bicep/tricep activity and optitract finger position, elbow angle, elbow angukar velocity vs. time. 
 
% Bicep/Tricep EMG Activity
%Load EMG file
 
FS = 1000;
FS1=120;
data = open('C:\Users\jadam\Desktop\NJIT\Spring 2022\BME384\exp1wide.mat');addpath 'C:\Users\jadam\Desktop\NJIT\Spring 2022\BME384\Data Lab 2';
EMGwide = data.ch1data;
Time = linspace(0,length(EMGwide)/FS,length(EMGwide));
[B,A] = butter(2,[20/(FS/2) 350/(FS/2)], 'bandpass');
EMFwidefilt=filtfilt(B,A,EMGwide);
EMGwideRect = abs(EMFwidefilt-mean(EMFwidefilt));
windowlength = 30; overlap = 29;
EMGwideEnv = rms1(EMGwideRect,windowlength,overlap,1);
 
% Finger Position
 
plot3(XD1,YD1,ZD1,'b'); 
hold on
plot3(XD5,YD5,ZD5,'k');
plot3(XD3,YD3,ZD3,'g');
plot3(XD2,YD2,ZD2,'r');
plot3(XD4,YD4,ZD4,'y');
xlabel('X-Coordinate(m)');
ylabel('Y-Coordinate(m)');
title('Arm Movement: Task B');
 
% Animate the stick figure for 2 D
for i=1:40:length(XD2)
    view([0 0 1]);
    h1 = line([XD3(i) XD1(i) XD5(i) XD2(i)], [YD3(i) YD1(i) YD5(i) YD2(i)], [ZD3(i) ZD1(i) ZD5(i) ZD2(i)]);
    h2 = plot3(XD2(i,1), YD2(i,1), ZD2(i,1), 'o','MarkerSize',10,'MarkerFaceColor','m');
    drawnow
    pause(0.1)
    delete(h1)
    delete(h2)
end
 
% Elbow Angle
D1=[XD1-XD5,YD1-YD5,ZD1-ZD5];
D2=[XD3-XD1,YD3-YD1,ZD3-ZD1];
ElbowAngleEyesClosed = [];
for i = 1:length(XD5)
    D1=[XD1(i)-XD5(i),YD1(i)-YD5(i),ZD1(i)-ZD5(i)];
    D2=[XD3(i)-XD1(i),YD3(i)-YD1(i),ZD3(i)-ZD1(i)];
    ThetaEyesClosed = acos(dot(D1,D2)./(norm(D1)*norm(D2)));
    ElbowAngleEyesClosed = [ElbowAngleEyesClosed;ThetaEyesClosed];
end
 
 
% Elbow Angular Velocity V. Time
AngularDisplacement = diff(XC1);
TimeChange = diff(TimeC);
Velocity = AngularDisplacement./TimeChange;
Velocity(end+1) = Velocity(end);
AngularDisplacement(end+1) = AngularDisplacement(end);
 
%plot
figure(21) 
subplot(2,2,1);
plot(Time, EMGwideRect);
title('EMG - Wide Target')
xlabel('Time(s)')
ylabel('EMG');
 
subplot(2,2,2);
plot3(XD1,YD1,ZD1,'b'); 
hold on
plot3(XD5,YD5,ZD5,'k');
plot3(XD3,YD3,ZD3,'g');
plot3(XD2,YD2,ZD2,'r');
plot3(XD4,YD4,ZD4,'y');
xlabel('X-Coordinate(m)');
ylabel('Y-Coordinate(m)');
title('Arm Movement: Task B');
view([0 0 1]);
line([XD3(1) XD1(1) XD5(1) XD2(1)], [YD3(1) YD1(1) YD5(1) YD2(1)], [ZD3(1) ZD1(1) ZD5(1) ZD2(1)]);
 
subplot(2,2,3)
D1=[XD1-XD5,YD1-YD5,ZD1-ZD5];
D2=[XD3-XD1,YD3-YD1,ZD3-ZD1];
ElbowAngleEyesClosed = [];
for i = 1:length(XD5)
    D1=[XD1(i)-XD5(i),YD1(i)-YD5(i),ZD1(i)-ZD5(i)];
    D2=[XD3(i)-XD1(i),YD3(i)-YD1(i),ZD3(i)-ZD1(i)];
    ThetaEyesClosed = acos(dot(D1,D2)./(norm(D1)*norm(D2)));
    ElbowAngleEyesClosed = [ElbowAngleEyesClosed;ThetaEyesClosed];
end
 
plot(TimeD,ElbowAngleEyesClosed);
xlabel('Time (Sec)');
ylabel('Elbow Angle (Theta)');
title('Elbow Angle for Eyes Closed');
 
subplot(2,2,4)
AngularDisplacement = diff(XC1);
TimeChange = diff(TimeC);
Velocity = AngularDisplacement./TimeChange;
Velocity(end+1) = Velocity(end);
AngularDisplacement(end+1) = AngularDisplacement(end);
 
plot(TimeC, Velocity)
xlabel('Time(s)');
ylabel('Angular Velocity (rad/s)');
title('Angular Velocity VS Time');