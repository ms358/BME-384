clc
clear
close all
%%
global subplot_position;
subplot_position = 1;
% [fingertip1, TimeA, V1A, V2A] = phonk("Data 2/Sahmet1A.csv", "wrist", "fingertip", "shoulder", "elbow", 2, 1, 1);
% [fingertip2, TimeB, V1B, V2B] = phonk("Data 2/Sahmet1B.csv", "elbow", "wrist", "fingertip", "shoulder", 3, 2, 2);
% [fingertip3, TimeC, V1C, V2C, targetc] = phonk("Data 2/Sahmet2C.csv", "target", "shoulder", "fingertip", "wrist", 3, 3, 3, "elbow", 1);
% [fingertip4, TimeD1, V1D, V2D, targetd] = phonk("Data 2/Sahmet2DTarget1.csv", "elbow", "shoulder", "target", "fingertip", 4, 4, 4, "wrist", 3);
[fingertip5, TimeD2, V1D2, V2D2, targetd2] = phonk("Data 2/Sahmet2DTarget2.csv", "shoulder", "elbow", "target", "wrist", 5, 5, 5, "fingertip", 3);
%%
fingertip = fingertip5;
Time = TimeD2;
filterfactor = 5;  % Example value; adjust according to your context

xval = fingertip(:,1);
yval = fingertip(:,2);
zval = fingertip(:,3);

velt = diff(Time);

if any(velt == 0)
    error('Consecutive identical time points detected. Velocity cannot be computed as division by zero occurs.');
end

velx = diff(xval)./velt;
vely = diff(yval)./velt;
velz = diff(zval)./velt;

% Handle potential Inf or NaN values after division
velx(~isfinite(velx)) = 0;  % Replace non-finite values with zero or other appropriate values

% Ensure vectors are column vectors
velx = [velx; velx(end)];
vely = [vely; vely(end)];
velz = [velz; velz(end)];

figure(7)
plot(Time(1:end), velx)  % Adjust time vector for one less element
legend("Velocity 5")

% Assuming a known sampling rate fs
fs = 60;  % Adjust this to match your actual sampling rate
[A, B] = butter(4, filterfactor/(fs/2), 'low');  % Correct cutoff frequency calculation
velx = filtfilt(A, B, velx);  % Apply filter

% Plot filtered velocity
figure(8)
plot(Time(1:end), velx)  % Adjust time vector for one less element
legend('Filtered Velocity 5')
