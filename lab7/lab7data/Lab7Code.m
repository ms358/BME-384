clc
clear
close all
Sub1 = readtable("Lab7- Task 1_Sub1.csv");
Sub2 = readtable("Lab7- Task 2_Sub 1.csv"); % i got hungwy

Sub1(1:4,:) = [];
Sub2(1:6,:) = [];
Sub2(:,12:end) = [];
Sub1 = table2array(Sub1);
Sub2 = table2array(Sub2);

%% Part 1 (eyes open)
close all
Time = Sub1(:,2);
Position = Sub1(:,3:end);
for k = 1:size(Position, 2)
    emptydata = isnan(Position(:, k));
    if any(emptydata)
        validdata = ~emptydata;
        validTime = Time(validdata);
        validPosition = Position(validdata, k);
        Position(:, k) = interp1(validTime, validPosition, Time, 'pchip', 'extrap');
    end
end
X1 = Position(:,1);
Y1 = Position(:,2);
Z1 = Position(:,3);
X2 = Position(:,4);
Y2 = Position(:,5);
Z2 = Position(:,6);
X3 = Position(:,7);
Y3 = Position(:,8);
Z3 = Position(:,9);
figure
hold on
plot3(X1,Y1,Z1,'k-'); % pointerfinger
plot3(X2,Y2,Z2,'Color','m'); % thumb
plot3(X3,Y3,Z3,'c-'); % wrist
hold off
aperturex = X1-X2;
aperturey = Y1-Y2;
aperturez = Z1-Z2;
magaperture = sqrt(aperturex.^2+aperturey.^2+aperturez.^2);
dWristypos = diff(Z3);
dTime = diff(Time);
Wristvelz = dWristypos./dTime;
Wristvelz = Wristvelz';
Wristvelz = [Wristvelz(end),Wristvelz];
Wristvelz = Wristvelz';

dWristxpos = diff(X3);
Wristvelx = dWristxpos./dTime;
Wristvelx = Wristvelx';
Wristvelx = [Wristvelx(end),Wristvelx];
Wristvelx = Wristvelx';

dWristxpos = diff(Y3);
Wristvely = dWristxpos./dTime;
Wristvely = Wristvely';
Wristvely = [Wristvely(end),Wristvely];
Wristvely = Wristvely';

%halving code
halftime = 2289;
Wristvelx2 = Wristvelx(halftime:end,1);
Wristvely2 = Wristvely(halftime:end,1);
Wristvelz2 = Wristvelz(halftime:end,1);
Time2 = Time(halftime:end,1);

Wristvelx = Wristvelx(1:halftime,1);
Wristvely = Wristvely(1:halftime,1);
Wristvelz = Wristvelz(1:halftime,1);
Time = Time(1:halftime,1);


% SIDE GRIP STUFF

figure
plot(Time,Wristvelz) % plots the "transport velocity"
[A, B] = butter(2, 2/60, "low");
Wristvelz = filtfilt(A, B, Wristvelz);
turningPoints = [];
i = 1;
while i <= length(Wristvelz) - 1
    % Check if current time is within desired range before evaluating Wristvelz
    if Time(i) > 6 && Time(i) < 38
        if abs((Wristvelz(i)-Wristvelz(i+10))) >= 0.02||abs((Wristvelz(i)-Wristvelz(i-10))) >= 0.02
            if (Wristvelz(i) <= 0.01 && Wristvelz(i + 1) > -0.01)||(Wristvelz(i) >= -0.01 && Wristvelz(i + 1) < 0.01) % Condition for turning point
                turningPoints = [turningPoints; i + 1]; % Collect turning point
                i = i + 80; %Search for turning Points in set interval
                continue; 
            end
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end

% Plot Y-velocity over time
figure;
plot(Time, Wristvelz, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
scatter(Time(turningPoints), Wristvelz(turningPoints), 'r', 'filled');

turningPointsX = Time(turningPoints(1:end));
turningPointsX = diff(turningPointsX);


% labeling
legend('X-Velocity', 'Turning Points');

figure;
hold on;

i1 = 1;
addedLegend1 = false;
for i2 = 2:3:length(turningPoints)
    h1 = plot3(X3(turningPoints(i1):turningPoints(i2)), ...
               Y3(turningPoints(i1):turningPoints(i2)), ...
               Z3(turningPoints(i1):turningPoints(i2)), 'Color','r');
    if ~addedLegend1
        set(h1, 'DisplayName', 'Subject 1');
        addedLegend1 = true;
    else
        set(h1, 'HandleVisibility', 'off');
    end
    i1 = i1+3;
end
legend show;
hold off
ylabel("anterior displacement (m)")
xlabel("lateral displacement (m)")

magnV = sqrt(Wristvelx.^2+Wristvely.^2+Wristvelz.^2);
magnV = smoothdata(magnV, "movmean", 20);

magaperture2 = magaperture(halftime:end,1);
magaperture = magaperture(1:halftime,1);

figure
hold on
yyaxis left
plot(Time, magnV)
ylabel('Velocity')
yyaxis right
plot(Time, magaperture)
ylabel('Distance')
title('Wrist Velocity Plot')
legend('Magnitude Velocity', 'aperture')
hold off

% MID GRIP
figure
plot(Time2,Wristvelz2) % plots the "transport velocity"
[A, B] = butter(2, 2/60, "low");
Wristvelz2 = filtfilt(A, B, Wristvelz2);
turningPoints = [];
i = 1;
while i <= length(Wristvelz2) - 1
    % Check if current time is within desired range before evaluating Wristvelz
    if Time2(i) > 6 && Time2(i) < 38
        if abs((Wristvelz2(i)-Wristvelz2(i+10))) >= 0.02||abs((Wristvelz2(i)-Wristvelz2(i-10))) >= 0.02
            if (Wristvelz2(i) <= 0.01 && Wristvelz2(i + 1) > -0.01)||(Wristvelz2(i) >= -0.01 && Wristvelz2(i + 1) < 0.01) % Condition for turning point
                turningPoints = [turningPoints; i + 1]; % Collect turning point
                i = i + 80; %Search for turning Points in set interval
                continue; 
            end
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end

% Plot Y-velocity over time
figure;
plot(Time2, Wristvelz2, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
scatter(Time2(turningPoints), Wristvelz2(turningPoints), 'r', 'filled');

turningPointsX = Time2(turningPoints(1:end));
turningPointsX = diff(turningPointsX);


% labeling
legend('X-Velocity', 'Turning Points');

figure;
hold on;

i1 = 1;
addedLegend1 = false;
for i2 = 2:3:length(turningPoints)
    h1 = plot3(X3(turningPoints(i1):turningPoints(i2)), ...
               Y3(turningPoints(i1):turningPoints(i2)), ...
               Z3(turningPoints(i1):turningPoints(i2)), 'Color','r');
    if ~addedLegend1
        set(h1, 'DisplayName', 'Subject 1');
        addedLegend1 = true;
    else
        set(h1, 'HandleVisibility', 'off');
    end
    i1 = i1+3;
end
legend show;
hold off
ylabel("anterior displacement (m)")
xlabel("lateral displacement (m)")

magnV = sqrt(Wristvelx.^2+Wristvely.^2+Wristvelz.^2);
magnV = smoothdata(magnV, "movmean", 20);

figure
hold on
yyaxis left
plot(Time, magnV)
ylabel('Velocity')
yyaxis right
plot(Time, magaperture)
ylabel('Distance')
title('Wrist Velocity Plot')
legend('Magnitude Velocity', 'aperture')
hold off


%%
clc
clear
close all

load('Lab7-Task 1 Dyna Subject 1.mat')
ForceData = ch2data;
[A, B] = butter(2, 25/500, "low");
ForceData = filtfilt(A, B, ForceData);
Time = linspace(1,35,35000);
figure
plot(Time,ForceData)
PositiveForceData = ForceData + abs(mean(ForceData));
threshold = 0.3 * max(PositiveForceData); 
figure
plot(Time,PositiveForceData)
onsets = find(diff(PositiveForceData > threshold) == 1);
offsets = find(diff(PositiveForceData > threshold) == -1);
squeezes = cell(length(onsets), 1);
for i = 1:length(onsets)
    squeezes{i} = PositiveForceData(onsets(i):offsets(i));
end
peak_forces = zeros(length(squeezes), 1);
for i = 1:length(squeezes)
    peak_forces(i) = max(squeezes{i});
end

areas = zeros(length(squeezes), 1);
for i = 1:length(squeezes)
    f = @(x) interp1(1:length(squeezes{i}), squeezes{i}, x, 'linear', 'extrap');
    areas(i) = trapz(squeezes{i}); % Using trapezoidal integration
end

disp(areas)
figure;
plot(ForceData);
hold on;
scatter(onsets, ForceData(onsets), 'g');
scatter(offsets, ForceData(offsets), 'r');
title('Force Trace with Onsets (green) and Offsets (red)');
hold off;

% Histogram of Peak Forces
figure;
histogram(peak_forces);
title('Histogram of Peak Forces');

%% Part 2 (eyes closed)
close all
Time = Sub2(:,2);
Position = Sub2(:,3:end);
for k = 1:size(Position, 2)
    emptydata = isnan(Position(:, k));
    if any(emptydata)
        validdata = ~emptydata;
        validTime = Time(validdata);
        validPosition = Position(validdata, k);
        Position(:, k) = interp1(validTime, validPosition, Time, 'pchip', 'extrap');
    end
end
X1 = Position(:,1);
Y1 = Position(:,2);
Z1 = Position(:,3);
X2 = Position(:,4);
Y2 = Position(:,5);
Z2 = Position(:,6);
X3 = Position(:,7);
Y3 = Position(:,8);
Z3 = Position(:,9);
figure
hold on
plot3(X1,Y1,Z1,'k-'); %pointerfinger
plot3(X2,Y2,Z2,'Color','m'); %thumb
plot3(X3,Y3,Z3,'c-'); %wirst
hold off

aperturex = X1-X2;
aperturey = Y1-Y2;
aperturez = Z1-Z2;
magaperture = sqrt(aperturex.^2+aperturey.^2+aperturez.^2);
dWristypos = diff(Z3);
dTime = diff(Time);
Wristvelz = dWristypos./dTime;
Wristvelz = Wristvelz';
Wristvelz = [Wristvelz(end),Wristvelz];
Wristvelz = Wristvelz';

dWristxpos = diff(X3);
Wristvelx = dWristxpos./dTime;
Wristvelx = Wristvelx';
Wristvelx = [Wristvelx(end),Wristvelx];
Wristvelx = Wristvelx';

dWristxpos = diff(Y3);
Wristvely = dWristxpos./dTime;
Wristvely = Wristvely';
Wristvely = [Wristvely(end),Wristvely];
Wristvely = Wristvely';
figure
plot(Time,Wristvelz)
[A, B] = butter(2, 5/60, "low");
Wristvelz = filtfilt(A, B, Wristvelz);
turningPoints = [];
i = 1;
while i <= length(Wristvelz) - 1
    % Check if current time is within desired range before evaluating Wristvelz
    if Time(i) > 6 && Time(i) < 36.4
        if abs((Wristvelz(i)-Wristvelz(i+10))) >= 0.02||abs((Wristvelz(i)-Wristvelz(i-10))) >= 0.02
            if (Wristvelz(i) <= 0.01 && Wristvelz(i + 1) > -0.01)||(Wristvelz(i) >= -0.01 && Wristvelz(i + 1) < 0.01) % Condition for turning point
                turningPoints = [turningPoints; i + 1]; % Collect turning point
                i = i + 75; %Search for turning Points in set interval
                continue; 
            end
        end
    end
    i = i + 1; % Otherwise, proceed to the next index
end

% Plot Y-velocity over time
figure;
plot(Time, Wristvelz, 'LineWidth', 2);
xlabel('Time');
ylabel('Velocity in Y');
title('Y-Velocity Over Time');
hold on;

% Highlight turning points on the plot
scatter(Time(turningPoints), Wristvelz(turningPoints), 'r', 'filled');

turningPointsX = Time(turningPoints(1:end));
turningPointsX = diff(turningPointsX);


% labeling
legend('X-Velocity', 'Turning Points');

figure;
hold on;

i1 = 1;
addedLegend1 = false;
for i2 = 2:3:length(turningPoints)
    h1 = plot3(X3(turningPoints(i1):turningPoints(i2)), ...
               Y3(turningPoints(i1):turningPoints(i2)), ...
               Z3(turningPoints(i1):turningPoints(i2)), 'Color','r');
    if ~addedLegend1
        set(h1, 'DisplayName', 'Subject 1');
        addedLegend1 = true;
    else
        set(h1, 'HandleVisibility', 'off');
    end
    i1 = i1+3;
end
legend show;
hold off

magnV = sqrt(Wristvelx.^2+Wristvely.^2+Wristvelz.^2);
figure
hold on
yyaxis left
plot(Time, magnV)
ylabel('Velocity')
yyaxis right
plot(Time, magaperture)
ylabel('Distance')
title('Wrist Velocity Plot')
legend('Magnitude Velocity', 'aperture')
hold off
%%
close all

load('Lab7-Task 2 Dyna Subject 1.mat')
ForceData = ch2data;
[A, B] = butter(2, 25/500, "low");
ForceData = filtfilt(A, B, ForceData);
Time = linspace(1,35,35000);
figure
plot(Time,ForceData)
PositiveForceData = ForceData + abs(mean(ForceData));
threshold = 0.20 * max(PositiveForceData); 
figure
plot(Time,PositiveForceData)
onsets = find(diff(PositiveForceData > threshold) == 1);
offsets = find(diff(PositiveForceData > threshold) == -1);
squeezes = cell(length(onsets), 1);
for i = 1:length(onsets)
    squeezes{i} = PositiveForceData(onsets(i):offsets(i));
end
peak_forces = zeros(length(squeezes), 1);
for i = 1:length(squeezes)
    peak_forces(i) = max(squeezes{i});
end

areas = zeros(length(squeezes), 1);
for i = 1:length(squeezes)
    f = @(x) interp1(1:length(squeezes{i}), squeezes{i}, x, 'linear', 'extrap');
    areas(i) = trapz(squeezes{i}); % Using trapezoidal integration
end
disp(areas)
figure;
plot(ForceData);
hold on;
scatter(onsets, ForceData(onsets), 'g');
scatter(offsets, ForceData(offsets), 'r');
title('Force Trace with Onsets (green) and Offsets (red)');
hold off;

% Histogram of Peak Forces
figure;
histogram(peak_forces);
title('Histogram of Peak Forces');
