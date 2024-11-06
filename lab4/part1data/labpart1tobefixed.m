clc
clear 
close all
[XpA, YpA, TzA, meanXpA, meanYpA, stdXpA, stdYpA]= parseNplot("Part A","Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv",'Group1_BME384_partA_data.csv',10001,10);
[XpB, YpB, TzB, meanXpB, meanYpB, stdXpB, stdYpB]= parseNplot("Part B", "Group1_BME384_rawdata_NIDAQ_PCI-6221_22859738.csv","Group1_BME384_partB_.csv",10001,10);

% Optitrack data
% clc;
% clear;
LottaData = readtable("Group1_BME384_partB.csv");
Coords = table2array(LottaData);
Coords(1:4,:) = [];
Timeend = 1000;
TimePos = Coords(1:Timeend,2);
Position = Coords(1:Timeend,477:end);
ForcePlateTimeend = 10000;
ForcePlateTime = 1:ForcePlateTimeend;
optitime = 1:Timeend;
Xcoords = Position(:,1:3:end);
Ycoords = Position(:,2:3:end);
Zcoords = Position(:,3:3:end);
labels = cell(1,39);
frame = length(TimePos);
skipx = XpA(1:10:10001,:);
skipy = YpA(1:10:10001,:);
skipz = TzA(1:10:10001,:);
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

for frame = 1:10:frame
    % Update all points for the current frame
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
    plot(XpA, YpA, 'b.'); % Plotting Xp vs Yp
    title('Xp vs Yp')
    xlabel('Xp')
    ylabel('Yp')
    hold on;
figure;    
    % Option 1: Circle
    theta = linspace(0, 2*pi, 10001);
    radius = sqrt(stdXpA^2 + stdYpA^2);
    xCircle = radius * cos(theta) + meanXpA;
    yCircle = radius * sin(theta) + meanYpA;
    plot(xCircle, yCircle, 'r-', 'LineWidth', 2); % Plotting the circle
    axis equal