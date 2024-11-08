function [Xp,Yp, Tz, meanXp,meanYp, stdXp, stdYp] = parseNplot(Part, filename1, filename2, framecount, spacing)
% T data, whatever that means :/
T=[2.9235 .0089 .0083 -.0180 -.0425 .0273
.0093 2.9310 .0033 .0075 -.0157 -.0217
.0131 .0234 11.5395 .0057 .0321 .0268
.0002 .0002 .0066 1.2722 -.0010 -.0118
.0003 -.0003 .0016 -.0010 1.2695 -.0055
-.0048 .0021 .0008 -.0057 -.0057 .5898];
FandM = readtable(filename1);
Data = table2array(FandM);
% find means
means = zeros(1,6);
for i = 5:10
    means(i-4) = mean(Data(1:framecount,i));
end
PartA = readtable(filename2);
Adata = table2array(PartA);
PartAFandM = Adata(1:framecount,5:10) - means;
Time = linspace(0,spacing,framecount);

Forces = (PartAFandM*T')/0.04; % Converting To N*m
Fx = Forces(:,1);
Fy = Forces(:,2);
Fz = Forces(:,3);
Mx = Forces(:,4);
My = Forces(:,5);
Mz = Forces(:,6);
% i = 0;
% Filtering lol
SF = 1000;
order = 2;
Cutoff = 50/500;
[b, a] = butter(order, Cutoff, "low");
FilteredForces = zeros(framecount,6);
for i = 1:6
FilteredForces(:,i) = filtfilt(b, a, Forces(:,i));
end
FFx = FilteredForces(:,1);
FFy = FilteredForces(:,2);
FFz = FilteredForces(:,3);
FMx = FilteredForces(:,4);
FMy = FilteredForces(:,5);
FMz = FilteredForces(:,6);
Xp = (-FMy)./(FFz); %
Yp = (-FMx)./(FFz); %
Tz = (FMz)-(Xp).*(FFy)+(Yp).*(FFx); %
meanXp = mean(Xp); %
meanYp = mean(Yp); %
stdXp = std(Xp); %
stdYp = std(Yp); %
figure("NumberTitle","off","Name",Part)
subplot(3,1,1)
plot(Time,Xp)
ylabel('X Center of Pressure')
subplot(3,1,2)
plot(Time,Yp)
ylabel('Y Center of Pressure')
subplot(3,1,3)
plot(Time,Tz)
ylabel('Torque About Z Axis')
xlabel('Time(s)')