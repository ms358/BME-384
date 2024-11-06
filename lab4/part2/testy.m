function [domliftmean, domliftstd, ymean, zmean, ys, zs, turningPoints, dy] = testy(Position, time, minDelay, timeMin, timeMax, descriptor, taskname, Position2)
%for one of our trials, out position was filled with part with Data that
%wasnt picked up, so interpolation was needed to fill in the gaps
emptydata = isnan(Position);
if any(emptydata)
    validdata = ~emptydata;
    validTime = time(validdata);
    validPosition = Position(validdata);
    filledPosition = interp1(validTime, validPosition, time, 'pchip', 'extrap');
    Position = filledPosition;
end
dy = diff(Position); % Change in Z
dt = diff(time); % Change in time
global subplot_position; % loads the variable for the subplot position
% Calculate Y-velocity
vy = dy ./ dt;
turningPoints = [];
i = 1;
while i <= length(vy) - 1
    % Check if current time is within desired range before evaluating vy
    if time(i) > timeMin && time(i) < timeMax
        if (vy(i) <= 0 && vy(i + 1) > 0)||(vy(i) >= 0 && vy(i + 1) < 0) % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; %Search for turning Points in set interval
            continue; 
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end

% Extract every other value
everyOther = Position(turningPoints(1:2:end));
everyOther2 = Position2(turningPoints(1:2:end));
% Calculate the average
average = mean(everyOther);
average2 = mean(everyOther2);

% Extract every other value starting from the second element
everyOtherOpposite = flip(Position(turningPoints(2:2:end)));
everyOtherOpposite2 = flip(Position2(turningPoints(2:2:end)));
% Calculate the average
averageOpposite = mean(everyOtherOpposite);
averageOpposite2 = mean(everyOtherOpposite2);
TY = [average, average2];
TZ = [averageOpposite, averageOpposite2];
% turningPointsmean = plot(TY,TZ,'Color','g');


% Calculate the slope of the original line
slope = (TZ(2) - TZ(1)) / (TY(2) - TY(1));

% Calculate the slope of the perpendicular line
slope_perp = -1 / slope;

% find perpendicular line
Yint = (TZ(2)-TZ(1))/2 - slope_perp * (TY(2)-TY(1))/2;


timeForVy = time(2:end); % Drop the first time value
% Plot Y-velocity over time
figure(3);
subplot(4,2,subplot_position)
plot(timeForVy, vy, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
figure(3);
subplot(4,2,subplot_position)
scatter(timeForVy(turningPoints), vy(turningPoints), 'r', 'filled');

turningPointsX = timeForVy(turningPoints(1:end));
turningPointsX = diff(turningPointsX);
domliftmean = mean(turningPointsX);
domliftstd = std(turningPointsX);
fprintf('Mean of time at turning points for %s: %f seconds\n', descriptor, domliftmean);
fprintf('Standard deviation of time at turning points for %s: %f seconds\n', descriptor, domliftstd);

% labeling
legend('Y-Velocity', 'Turning Points');


% moving mean
% figure(4);
% subplot(4,2,subplot_position)
movingmean = movmean(Position,5);




% Labeling plot elements:
legend('error bars','moving mean');
% iterate subplot_position
subplot_position = subplot_position + 1;
% Plots moving standard deviation
figure(4);
subplot(4,1,floorDiv(subplot_position,2))
movingstd = movstd(Position,10);
errorbar(time,movingmean,movingstd)
hold on
plot(time,movingmean) % plotting moving mean on top so it's visible
title(taskname)


% midpointsPosition = [];
% for i = 1:length(turningPoints)-1
%     startPoint = turningPoints(i);
%     endPoint = turningPoints(i+1);
%     midPoint = floor((startPoint + endPoint) / 2);
% 
%     midpointsPosition(end+1) = Position(midPoint); 
% 
% end
% for i = 1:numel(everyOtherOpposite)
%     avgTime = floor(everyOtherOpposite(i) + everyOther(i)) / 2;
% 
%     % Calculate spatial distance
%     spatialDistances(i) = Position2(avgTime) - mean(TZ);
% end


%plotting trials in appropiate subplot
figure(5);
subplot(4,2,subplot_position-1)
hold on
%finds the max amount of data between turning points
sections = diff(turningPoints);
maxlength = max(sections);

ys = NaN(maxlength, length(turningPoints)-1);
zs = NaN(maxlength, length(turningPoints)-1);
%puts data between turning points into columns to find mean lines
for i = 1:length(turningPoints)-1
    a = 1:sections(i);
    b = linspace(1, sections(i), maxlength);
    ySection = Position(turningPoints(i):turningPoints(i+1)-1);
    zSection = Position2(turningPoints(i):turningPoints(i+1)-1);
    % if odd, puts the data normally, if even, puts data in reverse, so
    % mean calculation is possible
    if mod(i,2) == 1
        %interpolates data so all data points are the same size 
        ys(:,i) = interp1(a, ySection, b, 'pchip', 'extrap');
        zs(:,i) = interp1(a, zSection, b, 'pchip', 'extrap');
    else
        yflipped = flip(ySection);
        zflipped = flip(zSection);
        ys(:,i) = interp1(a, yflipped, b, 'pchip', 'extrap');
        zs(:,i) = interp1(a, zflipped, b, 'pchip', 'extrap');
    end
end
% Assume TY, TZ define the reference line
slope = (TZ(2) - TZ(1)) / (TY(2) - TY(1));
intercept = TZ(1) - slope * TY(1);

numColumns = size(ys, 2); % Number of columns in ys and zs
allDistances = []; 

for col = 1:numColumns
    for row = 1:size(ys, 1)
        x0 = ys(row, col); % x-coordinate of the point
        y0 = zs(row, col); % y-coordinate of the point
        % Skip NaN values

        % Distance from point to line
        distance = abs(-slope * x0 + 1 * y0 - intercept) / sqrt(slope^2 + 1^2);
        allDistances = [allDistances, distance]; % Append distance
    end
end

% calculate the standard deviation of all distances
SpacialVar = std(allDistances);
fprintf('Spatial Variation: %f\n', SpacialVar);

x = linspace(TY(1),TY(2),maxlength);
Perp = slope_perp*x+Yint;
Perp = Perp';
% Perpen = plot(x,Perp,"Color",'y');
%zeros the points
for i = 1:size(ys, 2)
    yzeroed = ys(:,i) - ys(1,i);
    zzeroed = zs(:,i) - zs(1,i);
    ys(:,i) = yzeroed;
    zs(:,i) = zzeroed;
end
%rotating the polot

for i = 1:size(ys,2)
    ySection = ys(:,i);
    zSection = zs(:,i);
    diffy = ySection(end) - ySection(1);
    diffz = zSection(end) - zSection(1);
    angle = atan2(diffz, diffy);
    rotation = -rad2deg(angle) + 180; 
    midPoint = [(ySection(1) + ySection(end))/2, (zSection(1) + zSection(end))/2];
    ySection = ySection - midPoint(1);
    zSection = zSection - midPoint(2);
    h = plot(ySection, zSection, 'b');
    rotate(h, [0 0 1], rotation, [0 0 0]);
end
%rotating the mean line
ymean = mean(ys, 2);
zmean = mean(zs, 2);
mdiffy = ymean(end) - ymean(1);
mdiffz = zmean(end) - zmean(1);
angle = atan2(mdiffz, mdiffy);
rotation = -rad2deg(angle) + 180;
midPoint = [(ymean(1) + ymean(end))/2, (zmean(1) + zmean(end))/2];
ymean = ymean - midPoint(1);
zmean = zmean - midPoint(2);
h = plot(ymean, zmean, 'LineWidth', 3, 'Color', 'r');
rotate(h, [0 0 1], rotation, [0 0 0]);
% TY = TY - TY(1);
% TZ = TZ - TZ(1);
TY = [mean(ys(1,:)), mean(ys(end,:))];
TZ = [mean(zs(1,:)), mean(zs(end,:))];
%rotating turningPoint line
mdiffy = TY(2) - TY(1);
mdiffz = TZ(2) - TZ(1);
angle = atan2(mdiffz, mdiffy);
rotation = -rad2deg(angle) + 180;
midPoint = [(TY(1) + TY(end))/2, (TZ(1) + TZ(end))/2];
TY = TY - midPoint(1);
TZ = TZ - midPoint(2);
h = plot(TY, TZ, 'LineWidth', 1, 'Color', 'g');
rotate(h, [0 0 1], rotation, [0 0 0]);
TY = TY - TY(1);
TZ = TZ - TZ(1);

%rotating Perp Line
% mdiffy = x(end) - x(1);
% mdiffz = Perp(end) - Perp(1);
% angle = atan2(mdiffz, mdiffy);
rotation = -rad2deg(angle) + 90;
% midPoint = [(x(1) + x(end))/2, (Perp(1) + Perp(end))/2];
% x = x - midPoint(1);
% Perp = Perp - midPoint(2);

h = plot(TY, TZ, 'LineWidth', 2, 'Color', 'y');
rotate(h, [0 0 1], rotation, [0 0 0]);


hold off