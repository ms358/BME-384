function [accelmag, time] = idle(filename,endTime, Xp, Yp, Tz, stdXp, stdYp, meanXp, meanYp, lastframe)
global accelmag;
LottaData = readtable(filename);
Coords = table2array(LottaData);
Coords(1:4,:) = [];
TimePos = Coords(1:endTime,2);
Position = Coords(1:endTime,477:end);
Xcoords = Position(:,1:3:end);
Ycoords = Position(:,2:3:end);
Zcoords = Position(:,3:3:end);
frame = length(TimePos);
skipx = Xp(1:10:lastframe,:);
skipy = Yp(1:10:lastframe,:);
skipz = Tz(1:10:lastframe,:);
figure;
subplot(1,4,1)
hold on;
dude = plot3(Xcoords(1,:), Ycoords(1,:), Zcoords(1,:), 'o', 'MarkerSize', 3);
hip = line([Xcoords(1,21), Xcoords(1,30), Xcoords(1,12), Xcoords(1,4)], [Ycoords(1,21), Ycoords(1,30), Ycoords(1,12), Ycoords(1,4)], [Zcoords(1,21), Zcoords(1,30), Zcoords(1,12), Zcoords(1,4)], 'Color', 'b');
sternum = line([Xcoords(1,13), Xcoords(1,2), Xcoords(1,31)], [Ycoords(1,13), Ycoords(1,2), Ycoords(1,31)], [Zcoords(1,13), Zcoords(1,2), Xcoords(1,31)], 'Color', 'b');
axis equal;
subplot(1,4,2)
barHandlex = bar(skipx);
legend('Xp');
subplot(1,4,3)
barHandley = bar(skipy);
legend('Yp');
subplot(1,4,4)
barHandlez = bar(skipz);
legend('Tz');

for frame = 1:50:frame
    % constantly plots frames
    set(dude, 'XData', Xcoords(frame,:), 'YData', Ycoords(frame,:), 'ZData', Zcoords(frame,:));
    set(hip, 'XData', [Xcoords(frame,21), Xcoords(frame,30), Xcoords(frame,12), Xcoords(frame,4)], 'YData', [Ycoords(frame,21), Ycoords(frame,30), Ycoords(frame,12), Ycoords(1,4)], 'ZData', [Zcoords(frame,21), Zcoords(frame,30), Zcoords(frame,12), Zcoords(frame,4)]);
    set(sternum, 'XData', [Xcoords(frame,13), Xcoords(frame,2), Xcoords(frame,31)], 'YData', [Ycoords(frame,13), Ycoords(frame,2), Ycoords(frame,31)], 'ZData', [Zcoords(frame,13), Zcoords(frame,2), Xcoords(frame,31)]);

    subplot(1, 4, 2);
    set(barHandlex, 'YData', skipx(frame,:))
     subplot(1, 4, 3);
    set(barHandley, 'YData', skipy(frame,:))
     subplot(1, 4, 4);
    set(barHandlez, 'YData', skipz(frame,:))

    drawnow; % Update the figure with new data
end



legend
% legend(labels, 'Location', 'bestoutside')
hold off;

figure;
    plot(Xp, Yp, 'b.'); % Plotting Xp vs Yp
    title('Xp vs Yp')
    xlabel('Xp')
    ylabel('Yp')
    hold on;    
    theta = linspace(0, 2*pi, 10001);
    radius = sqrt(stdXp^2 + stdYp^2);
    xCircle = radius * cos(theta) + meanXp;
    yCircle = radius * sin(theta) + meanYp;
    plot(xCircle, yCircle, 'r-', 'LineWidth', 2);
    axis equal
figure;
    poositionX=Xcoords(1:endTime,8);
    poositionY=Ycoords(1:endTime,8);
    poositionZ=Zcoords(1:endTime,8);
    velx = diff(poositionX);
    vely=diff(poositionY);
    velz=diff(poositionZ);
    accelx=diff(velx);
    accely=diff(vely);
    accelz=diff(velz);
accelmag=sqrt(accelx.^2+accely.^2+accelz.^2);
time = linspace(1,endTime,600);
[b, a] = butter(2, 50/500, "low");
accelmag=filtfilt(b, a, accelmag);
yyaxis left
plot(1:endTime-2,accelmag) 
hold on
lxp = length(Xp);
timelengthimadeup = endTime + 1;
yyaxis right
plot(1:timelengthimadeup,Xp(1:lxp/timelengthimadeup:lxp,1))
title('Acceleration of Hand Chart')
xlabel('time')
ylabel('Acceleration of Hand')