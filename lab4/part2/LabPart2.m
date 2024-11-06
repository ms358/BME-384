clc
clear vars
close all
%% Part 1 Stuff
% This is Bryan's signature with dominant hand (left)
figure(1)
subplot(2,1,1)
handcoords = readtable("bryan dominant.csv");
handcoords(1:4,:)=[]; % make top 4 rows empty
data=table2array(handcoords);
data = data*100; % converts to centimeters
hold on
thumb = [data(:,9),data(:,10),data(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("pen");
title('dominant hand')
hold off
axis equal
subplot(2,1,2)
% This is Bryan's Signature with nondominant hand (right)
handcoords = readtable("bryan non-dominant.csv");
handcoords(1:4,:)=[]; % make top 4 rows empty
data=table2array(handcoords);
data = data*100; % converts to centimeters
hold on
thumb = [data(:,9),data(:,10),data(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("pen");
title('non-dominant hand')
hold off
axis equal
%% PART 2 STUFF
%% TASK A
% Lifting Dominant Hand
clc
global subplot_position;
subplot_position = 1;
% clear vars
% close all
liftcoords = readtable("jon dominant lifting.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
figure(2);
subplot(4,2,1)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
% thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('dom hand lifting')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 50, 0, 11.5, 'Lifting Domininant Hand', "Task A", wrist(:,3));

% Lifting NONDominant Hand
figure(2)
% clear vars
liftcoords = readtable("jon non dominant lifting.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,2)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % wrist 
hold on
% wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
% plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("pen");
title('non-dom hand lift')
hold off
axis equal
[meanTime, stdTime] = testy(thumb(:,2), time, 140/2, 0.3, 13, 'Lifting NonDomininant Hand', "Task A", thumb(:,3));
%% TASK B
% Drag Dominant Hand
figure(2);
% clear vars
dragcoords = readtable("jon dominant dragg.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,3)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
% thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('dom hand drag')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 10, 0.5/2, 4, 'Drag Dominant Hand', "Task B", wrist(:,3));
% Drag NONDominant Hand
figure(2)
% clear vars
dragcoords = readtable("jon non dominant dragging.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,4)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % wrist
hold on
% wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
% plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("pen");
title('non-dom hand drag')
hold off
axis equal
[meanTime, stdTime] = testy(thumb(:,2), time, 20/2, 4, 7.4, 'Drag NonDominant Hand', "Task B", thumb(:,3));
%% TASK C
% Lifting Dominant Hand Thin
figure(2)
% clear vars
liftcoords = readtable("jon dominant lifting thin.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,5)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
% thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('dom hand lift thin')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 120/2, 1, 14, 'Lifting Dominant Hand Thin', "Task C", wrist(:,3));
% Lifting NONDominant Hand Thin
figure(2)
% clear vars
liftcoords = readtable("jon non dominant lifting thin.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,6)
plot3(pen(:,1),pen(:,2),pen(:,3)); % pen
hold on
% wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
% plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
% thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('non-dom hand lift thin')
hold off
axis equal
[meanTime, stdTime] = testy(pen(:,2), time, 50, 0.3, 10, 'Lifting NonDominant Hand Thin', "Task C", pen(:,3));
%% TASK D
% Drag Dominant Hand Thin
figure(2);
% clear vars
dragcoords = readtable("jon dominant dragg thin.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
% pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,7)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
% thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('dom hand drag thin')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 40/2, 1, 6.2, 'Drag Dominant Hand Thin', "Task D", wrist(:,3));
% Drag NONDominant Hand Thin
figure(2)
%clear vars
dragcoords = readtable("jon non dominant dragg thin.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,8)
% plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
% thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
% plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen");
title('non-dom hand drag thin')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 17, 0.15, 3.7, 'Drag Non-Dominant Hand Thin', "Task D", wrist(:,3)); % using 17
%instead of 17.5 here because 35/2 causes an error (can't have half a frame)
