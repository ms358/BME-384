function [turningPoints, velx, vely, velz] = turningpoints(fingertip, Time, filterfactor, starttime, endtime, minDelay)
global subplot_position;
% find velocities and plot x velocity
xval = fingertip(:,1);
yval = fingertip(:,2);
zval = fingertip(:,3);
velt = diff(Time);
velx = diff(xval)./velt;
vely = diff(yval)./velt;
velz = diff(zval)./velt;
velx = velx';
vely = vely';
velz = velz';
velx = [velx(end),velx];
vely = [vely(end),vely];
velz = [velz(end),velz];
velx = velx';
vely = vely';
velz = velz';
figure(7)
subplot(3,2,subplot_position)
plot(Time,velx)
legend("Velocity (condition " + subplot_position+")")
% filtering
velx(~isfinite(velx)) = 0; % replace weird values with 0
[A, B] = butter(2, filterfactor/60, "low");
velx = filtfilt(A, B, velx);
timeForVelx = Time(1:end); % Drop the first time value
% plotting lines
figure(8);
subplot(3,2,subplot_position)
plot(timeForVelx, velx, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;
% find turningPoints
turningPoints = [];
i = 1;
while i <= length(velx) - 1
    % Check if current time is within desired range before evaluating velC
    if Time(i) > starttime && Time(i) < endtime
        if (velx(i) <= 0 && velx(i + 1) > 0)||(velx(i) >= 0 && velx(i + 1) < 0) % Condition for turning point
            turningPoints = [turningPoints; i + 1]; % Collect turning point
            i = i + minDelay; %Search for turning Points in set interval
            continue; 
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end
% Extract every other value
everyOther = xval(turningPoints(1:2:end));
everyOther2 = yval(turningPoints(1:2:end));
everyOther3 = zval(turningPoints(1:2:end));
everyOtherOpposite = flip(xval(turningPoints(2:2:end)));
everyOtherOpposite2 = flip(yval(turningPoints(2:2:end)));
everyOtherOpposite3 = flip(zval(turningPoints(2:2:end)));
% Highlight turning points on the plot
figure(8);
scatter(timeForVelx(turningPoints), velx(turningPoints), 'r', 'filled');

turningPointsX = timeForVelx(turningPoints(1:end));
turningPointsX = diff(turningPointsX);


% new
% legend('X-Velocity', 'Turning Points');
% 
% BRYAN PLEEEEEZE
% 
% figure;
% hold on;
% 
% i1 = 1;
% addedLegend1 = false;
% for i2 = 2:4:length(turningPoints)
%     h1 = plot3(fingertip(turningPoints(i1):turningPoints(i2),1), ...
%                fingertip(turningPoints(i1):turningPoints(i2),2), ...
%                fingertip(turningPoints(i1):turningPoints(i2),3), 'Color','r');
%     if ~addedLegend1
%         set(h1, 'DisplayName', 'Target 1');
%         addedLegend1 = true;
%     else
%         set(h1, 'HandleVisibility', 'off');
%     end
%     i1 = i1+4;
% end
% legend show;
% hold off
% figure
% hold on
% i1 = 3;
% addedLegend2 = false;
% for i2 = 4:4:length(turningPoints)
%     h2 = plot3(fingertip(turningPoints(i1):turningPoints(i2),1), ...
%                fingertip(turningPoints(i1):turningPoints(i2),2), ...
%                fingertip(turningPoints(i1):turningPoints(i2),3), 'Color','b');
%     if ~addedLegend2
%         set(h2, 'DisplayName', 'Target 2');
%         addedLegend2 = true;
%     else
%         set(h2, 'HandleVisibility', 'off');
%     end
%     i1 = i1+4;
% end
% legend show;
% hold off
% endpointfinger = turningPoints(1:2:end);
% for i = 1:length(endpointfinger)
%     epf(i,1) = fingertip(endpointfinger(i),1);
%     epf(i,2) = fingertip(endpointfinger(i),2);
%     epf(i,3) = fingertip(endpointfinger(i),3);
% end

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

subplot_position = subplot_position + 1;