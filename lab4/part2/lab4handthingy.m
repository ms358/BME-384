clc
clear vars
close all
%% Part 1 Stuff
% This is Bryan's signature with dominant hand (left)
f1 = figure;
subplot(2,1,1)
handcoords = readtable("bryan dominant.csv");
handcoords(1:4,:)=[]; % make top 4 rows empty
data=table2array(handcoords);
data = data*100; % converts to centimeters
pen = [data(:,3),data(:,4),data(:,5)];
plot3(pen(:,1),pen(:,2),pen(:,3)); % wrist
hold on
wrist = [data(:,6),data(:,7),data(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [data(:,9),data(:,10),data(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("wrist","thumb","pen");
hold off
axis equal
subplot(2,1,2)
% This is Bryan's Signature with nondominant hand (right)
handcoords = readtable("bryan non-dominant.csv");
handcoords(1:4,:)=[]; % make top 4 rows empty
data=table2array(handcoords);
data = data*100; % converts to centimeters
pen = [data(:,3),data(:,4),data(:,5)];
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [data(:,6),data(:,7),data(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % wrist
thumb = [data(:,9),data(:,10),data(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("thumb","wrist","pen");
hold off
axis equal
%% Lifting Dominant Hand
f2 = figure;
clc
clear vars
close all
liftcoords = readtable("jon dominant lifting.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,1)
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("thumb","pen","wrist");
title('dom hand lifting')
hold off
axis equal
[meanTime, stdTime] = testy(wrist(:,2), time, 50, 0, 11.5);
figure
%%
% Drag Dominant Hand
clear vars
dragcoords = readtable("jon dominant dragg.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time2 = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,3)
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("thumb","pen","wrist");
title('dom hand drag')
hold off
axis equal

dy = diff(wrist(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;


minDelay = 10; % Minimum index gap between turning points
turningPoints = [];
i = 1;
for h = 1:length(time2)
    if time2(h) > 0.5 & time2(h) < 4
        i = h;
            if i <= length(vy) - 1 && time2(i) < 4
                if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
                    turningPoints = [turningPoints; i + 1]; % Collect turning point
                    i = i + minDelay; % Skip ahead by minDelay to enforce a gap
                else
                i = i + 1; % Otherwise, proceed to the next index
                end
            end
    end
end


% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end-1));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for dom hand drag: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for dom hand drag: %f seconds\n', domliftstd);
% Lifting Dominant Hand Thin
clear vars
liftcoords = readtable("jon dominant lifting thin.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time2 = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,5)
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("thumb","pen","wrist");
title('dom hand lift thin')
hold off
axis equal


dy = diff(pen(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 120; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 1 && time2(i) < 14
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for dom hand lift thin: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for dom hand lift thin: %f seconds\n', domliftstd);
% Drag Dominant Hand Thin
clear vars
dragcoords = readtable("jon dominant dragg thin.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time2 = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,7)
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen","thumb","wrist");
title('dom hand drag thin')
hold off
axis equal


dy = diff(wrist(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 40; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 1 && time2(i) < 6.2
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% % Adjust time vector for plotting against vy, which is one element shorter
% timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for dom hand drag thin: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for dom hand drag thin: %f seconds\n', domliftstd);
% Non Dominant hand
% Lifting Dominant Hand

clear vars

liftcoords = readtable("jon non dominant lifting.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time2 = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,2)
plot3(pen(:,1),pen(:,2),pen(:,3)); % wrist 
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("wrist","thumb","pen");
title('non-dom hand lift')
hold off
axis equal


dy = diff(wrist(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 140; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 0.3 && time2(i) < 13
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% Plot Y-velocity over time
f2 = figure;
plot(timeForVy, vy, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for non-dom hand lift: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for non-dom hand lift: %f seconds\n', domliftstd);
% Drag Dominant Hand
clear vars
dragcoords = readtable("jon non dominant dragging.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time2 = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,4)
plot3(pen(:,1),pen(:,2),pen(:,3)); % wrist
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % pen
legend("wrist","thumb","pen");
hold off
axis equal

dy = diff(thumb(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 20; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 4 && time2(i) < 7.4
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for non-dom hand drag: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for non-dom hand drag: %f seconds\n', domliftstd);
% Lifting Dominant Hand Thin
clear vars
liftcoords = readtable("jon non dominant lifting thin.csv");
liftcoords(1:4,:)=[]; % make top 4 rows empty
liftdata = table2array(liftcoords);
time2 = liftdata(:,2);
liftdata = liftdata*100; % converts to centimeters
pen = [liftdata(:,3),liftdata(:,4),liftdata(:,5)];
subplot(4,2,6)
plot3(pen(:,1),pen(:,2),pen(:,3)); % pen
hold on
wrist = [liftdata(:,6),liftdata(:,7),liftdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % thumb
thumb = [liftdata(:,9),liftdata(:,10),liftdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen","thumb","wrist");
hold off
axis equal

dy = diff(pen(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 100; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 0.3 && time2(i) < 10
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for non-dom hand lift: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for non-dom hand lift: %f seconds\n', domliftstd);
% Drag Dominant Hand Thin
clear vars
dragcoords = readtable("jon non dominant dragg thin.csv");
dragcoords(1:4,:)=[]; % make top 4 rows empty
dragdata = table2array(dragcoords);
time2 = dragdata(:,2);
dragdata = dragdata*100; % converts to centimeters
pen = [dragdata(:,3),dragdata(:,4),dragdata(:,5)];
subplot(4,2,8)
plot3(pen(:,1),pen(:,2),pen(:,3)); % thumb
hold on
wrist = [dragdata(:,6),dragdata(:,7),dragdata(:,8)];
plot3(wrist(:,1),wrist(:,2),wrist(:,3)); % pen
thumb = [dragdata(:,9),dragdata(:,10),dragdata(:,11)];
plot3(thumb(:,1),thumb(:,2),thumb(:,3)); % wrist
legend("pen","thumb","wrist");
hold off
axis equal

dy = diff(wrist(:,3)); % Change in Z
dt = diff(time2); % Change in time

% Calculate Y-velocity
vy = dy ./ dt;

minDelay = 35; % Minimum index gap between turning points
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time2 is within desired range before evaluating vy
    if time2(i) > 0.15 && time2(i) < 3.7
        if vy(i) <= 0 && vy(i + 1) > 0 % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; % Skip ahead by minDelay to enforce a gap
            continue; % Skip the rest of the loop and continue with the next iteration
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end



% Adjust time vector for plotting against vy, which is one element shorter
timeForVy = time2(2:end); % Drop the first time value
% % Plot Y-velocity over time
% figure;
% plot(timeForVy, vy, 'LineWidth', 2);
% xlabel('Time');
% ylabel('Velocity in Y');
% title('Y-Velocity Over Time');
% hold on;
% 
% % Highlight turning points on the plot
% scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');
% legend('Y-Velocity', 'Turning Points');
turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for non-dom hand drag: %f seconds\n', domliftmean);
fprintf('Standard deviation of time at turning points for non-dom hand drag: %f seconds\n', domliftstd);