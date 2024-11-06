clc;
clear;
close all;
%% plotting trajectories
global subplot_position;
subplot_position = 1;
[fingertip1, TimeA, V1A, V2A] = phonk("Data 2/Sahmet1A.csv", "wrist", "fingertip", "shoulder", "elbow", 2, 1, 1);
[fingertip2, TimeB, V1B, V2B] = phonk("Data 2/Sahmet1B.csv", "elbow", "wrist", "fingertip", "shoulder", 3, 2, 2);
[fingertip3, TimeC, V1C, V2C, targetc] = phonk("Data 2/Sahmet2C.csv", "target", "shoulder", "fingertip", "wrist", 3, 3, 3, "elbow", 1);
[fingertip4, TimeD1, V1D, V2D, targetd] = phonk("Data 2/Sahmet2DTarget1.csv", "elbow", "shoulder", "target", "fingertip", 4, 4, 4, "wrist", 3);
[fingertip5, TimeD2, V1D2, V2D2, targetd2] = phonk("Data 2/Sahmet2DTarget2.csv", "shoulder", "elbow", "target", "wrist", 5, 5, 5, "fingertip", 3);
%% cry. 😢
close all
subplot_position = 1;
[turningpointsA, velxA] = turningpoints(fingertip1,TimeA, 10, 0.95, 8.5, 48);
[turningpointsB, velxB] = turningpoints(fingertip2,TimeB, 10, 0.5, 9.3, 48);
[turningpointsC, velxC] = turningpoints(fingertip3,TimeC, 10, 2.4, 28.5, 75);
% gotta fine tune these later
[turningpointsD1, velxD1] = turningpoints(fingertip4,TimeD1, 1, 4.8, 72, 90);
[turningpointsD2, velxD2] = turningpoints(fingertip5,TimeD2, 1, 4, 92, 95);
%% find peaks (one must imagine sisyphus happy)
sisyphus(velxA, velyA, velzA, 0.3, 50);
sisyphus(velxB, velyB, velzB, 0.3, 50);
sisyphus(velxC, velyC, velzC, 0.3, 50); % C
sisyphus(velxD1, velyD1, velzD1, 0.3, 50);
sisyphus(velxD2, velyD2, velzD2, 0.3, 50);
%% Elbow Angle Vs Time and angular velocity

%%
Xvalue = C(:,1);
Yvalue = C(:,2);
Zvalue = C(:,3);
Cd_x = diff(C(:,1));
Cd_y = diff(C(:,2));
Cd_z = diff(C(:,3));
Cd_t = diff(TimeC);
velC = Cd_x./Cd_t;
velCy = Cd_y./Cd_t;
velCz = Cd_z./Cd_t;
velC = velC';
velCy = velCy';
velCz = velCz';
velC = [velC(end),velC];
velCy = [velCy(end),velCy];
velCz = [velCz(end),velCz];
velC = velC';
velCy = velCy';
velCz = velCz';
figure
plot(TimeC,velC)
[A, B1] = butter(2, 50/500, "low");
velC = filtfilt(A, B1, velC);
turningPoints = [];
i = 1;
while i <= length(velC) - 1
    % Check if current time is within desired range before evaluating velC
    if TimeC(i) > 2.4 && TimeC(i) < 28.5
        if (velC(i) <= 0 && velC(i + 1) > 0)||(velC(i) >= 0 && velC(i + 1) < 0) % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + 75; %Search for turning Points in set interval
            continue; 
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end
% Extract every other value
everyOther = Xvalue(turningPoints(1:2:end));
everyOther2 = Yvalue(turningPoints(1:2:end));
everyOther3 = Zvalue(turningPoints(1:2:end));
% Calculate the average
% average = mean(everyOther);
% average2 = mean(everyOther2);

% Extract every other value starting from the second element
everyOtherOpposite = flip(Xvalue(turningPoints(2:2:end)));
everyOtherOpposite2 = flip(Yvalue(turningPoints(2:2:end)));
everyOtherOpposite3 = flip(Zvalue(turningPoints(2:2:end)));
% Calculate the average
% averageOpposite = mean(everyOtherOpposite);
% averageOpposite2 = mean(everyOtherOpposite2);
% TY = [average, average2];
% TZ = [averageOpposite, averageOpposite2];
% turningPointsmean = plot(TY,TZ,'Color','g');



timeForvelC = TimeC(1:end); % Drop the first time value
% Plot Y-velocity over time
figure(3);
plot(timeForvelC, velC, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
figure(3);
scatter(timeForvelC(turningPoints), velC(turningPoints), 'r', 'filled');

turningPointsX = timeForvelC(turningPoints(1:end));
turningPointsX = diff(turningPointsX);


% labeling
legend('X-Velocity', 'Turning Points');

figure;
hold on;

i1 = 1;
addedLegend1 = false;
for i2 = 2:4:length(turningPoints)
    h1 = plot3(C(turningPoints(i1):turningPoints(i2),1), ...
               C(turningPoints(i1):turningPoints(i2),2), ...
               C(turningPoints(i1):turningPoints(i2),3), 'Color','r');
    if ~addedLegend1
        set(h1, 'DisplayName', 'Target 1');
        addedLegend1 = true;
    else
        set(h1, 'HandleVisibility', 'off');
    end
    i1 = i1+4;
end
legend show;
hold off
figure
hold on
i1 = 3;
addedLegend2 = false;
for i2 = 4:4:length(turningPoints)
    h2 = plot3(C(turningPoints(i1):turningPoints(i2),1), ...
               C(turningPoints(i1):turningPoints(i2),2), ...
               C(turningPoints(i1):turningPoints(i2),3), 'Color','b');
    if ~addedLegend2
        set(h2, 'DisplayName', 'Target 2');
        addedLegend2 = true;
    else
        set(h2, 'HandleVisibility', 'off');
    end
    i1 = i1+4;
end
legend show;
hold off
endpointfinger = turningPoints(1:2:end);
for i = 1:length(endpointfinger)
    epf(i,1) = C(endpointfinger(i),1);
    epf(i,2) = C(endpointfinger(i),2);
    epf(i,3) = C(endpointfinger(i),3);
end


%finds peaks
[Onepiece, locationx] = findpeaks(-velC, 'MinPeakHeight', 0.3, 'MinPeakDistance', 50);
figure;
hold on; 

% Plot velocity components
plot(1:length(velC), velC, 'b', 'DisplayName', 'Velocity Component X');
% Plot peak locations
plot(locationx, velC(locationx), 'bo', 'MarkerSize', 8, 'DisplayName', 'Peaks X');
xlabel('Time');
ylabel('Absolute Velocity');
title('Absolute Velocity and Peaks in X, Y, and Z Components');
hold off;
for i = 1:length(locationx)
    magnV(i) = sqrt(velC(i)^2+velCy(i)^2+velCz(i)^2);
end
meanPeakV = mean(magnV);
stdPeakV = std(magnV);
fprintf('Mean Peak Velocity is %d\n', meanPeakV)
fprintf('Standard Deviation is %d\n', stdPeakV)
