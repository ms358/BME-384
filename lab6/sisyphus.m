function [peaks] = sisyphus(velocityx, velocityy, velocityz, minpeakheight, minpeakdistance)
%finds peaks
[peaks, locationx] = findpeaks(-velocityx, 'MinPeakHeight', 0.3, 'MinPeakDistance', 50);
figure;
hold on;

% Plot velocity components
plot(1:length(velocityx), velocityx, 'b', 'DisplayName', 'Velocity Component X');
% Plot peak locations
plot(locationx, velocityx(locationx), 'bo', 'MarkerSize', 8, 'DisplayName', 'Peaks X');
xlabel('Time');
ylabel('Absolute Velocity');
title('Absolute Velocity and Peaks in X, Y, and Z Components');
hold off;
for i = 1:length(locationx)
    magnV(i) = sqrt(velocityx(i)^2+velocityy(i)^2+velocityz(i)^2);
end
meanPeakV = mean(magnV);
stdPeakV = std(magnV);
fprintf('Mean Peak Velocity is %d\n', meanPeakV)
fprintf('Standard Deviation is %d\n', stdPeakV)
