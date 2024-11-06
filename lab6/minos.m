function [] = minos()
close all
dot = sum(V1C .* V2C, 2);
magV1 = sqrt(sum(V1C.^2, 2));
magV2 = sqrt(sum(V2C.^2, 2));
angrads = acos(dot./(magV1.*magV2));
elbow_angle = angrads*(180/pi);
delbow_ang = diff(angrads);
angularvel = delbow_ang./Cd_t;
angularvel = angularvel';
angularvel = [angularvel(end),angularvel];
angularvel = angularvel';

bicepdata = load("Data 2\SahmetBicep2C.mat");
tricepdata = load("Data 2\SahmetTricep2C.mat"); 
biceps = bicepdata.ch2data;
triceps = tricepdata.ch1data;
biceps = [biceps(end),biceps];
triceps = [triceps(end),triceps];
biceps = biceps(1:10:end);
triceps = triceps(1:10:end);
figure
plot(TimeC, elbow_angle, 'b-');
title("Elbow Joint Angle vs Time");
xlabel('Time (s)');
ylabel('Elbow Angle (degrees)');
EMGTime = linspace(1,30,3001);
p = 1;
for i = 1:4:length(turningPoints)-1
    figure
    subplot(2,2,p)
    hold on
    plot(TimeC(turningPoints(i):turningPoints(i+1)), elbow_angle(turningPoints(i):turningPoints(i+1)), 'b-');
    title("Elbow Joint Angle vs Time (Target 1)");
    xlabel('Time (s)');
    ylabel('Elbow Angle (degrees)');
    hold off
    p = p+1;
    subplot(2,2,p) 
    hold on
    plot(TimeC(turningPoints(i):turningPoints(i+1)), angularvel(turningPoints(i):turningPoints(i+1)), 'b-');
    title("Elbow Joint Anglular Velocity vs Time (Target 1)");
    xlabel('Time (s)');
    ylabel('Angular Velocity (rads/s)');    
    hold off
    p = p+1;
    if turningPoints(i) <= 3000
    subplot(2,2,p)
    hold on
    plot(EMGTime(turningPoints(i):turningPoints(i+1)), biceps(turningPoints(i):turningPoints(i+1)), 'b-');
    plot(EMGTime(turningPoints(i):turningPoints(i+1)), triceps(turningPoints(i):turningPoints(i+1)), 'r-');
    title("EMGdata vs Time (Target 1)");
    xlabel('Time (s)');
    ylabel('Voltage');  
    legend('Biceps','Triceps')
    hold off
    else
        continue
    end
    p = 1;
end
p = 1;
for i = 3:4:length(turningPoints)-1
    figure
    subplot(2,2,p)
    hold on
    plot(TimeC(turningPoints(i):turningPoints(i+1)), elbow_angle(turningPoints(i):turningPoints(i+1)), 'b-');
    title("Elbow Joint Angle vs Time (Target 2)");
    xlabel('Time (s)');
    ylabel('Elbow Angle (degrees)');
    hold off
    p = p+1;
    subplot(2,2,p)
    hold on
    plot(TimeC(turningPoints(i):turningPoints(i+1)), angularvel(turningPoints(i):turningPoints(i+1)), 'b-');
    title("Elbow Joint Anglular Velocity vs Time (Target 2)");
    xlabel('Time (s)');
    ylabel('Angular Velocity (rads/s)');    
    hold off
    p = p+1;
    if turningPoints(i) <= 3000
    subplot(2,2,p)
    hold on
    plot(EMGTime(turningPoints(i):turningPoints(i+1)), biceps(turningPoints(i):turningPoints(i+1)), 'b-');
    plot(EMGTime(turningPoints(i):turningPoints(i+1)), triceps(turningPoints(i):turningPoints(i+1)), 'r-');
    title("EMGdata vs Time (Target 2)");
    xlabel('Time (s)');
    ylabel('Voltage');  
    legend('Biceps','Triceps')
    hold off
    else
        continue
    end
    p = 1;
end
figure
hold on
yyaxis left
plot(TimeC,angularvel, 'b-')
ylabel('Angular Velocity (rad/s)')
yyaxis right
plot(EMGTime,biceps, 'r-')
plot(EMGTime(1:length(triceps)),triceps, 'k-')
title('EMG Data & Angular Velocity vs Time')
ylabel('Voltage')
xlabel('Time (s)')
legend('Angular Velocity', 'Biceps Data', 'Triceps Data')