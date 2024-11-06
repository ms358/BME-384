%% loading initial stuff
clc;
clear variables;
close all;
load experimenta104.mat
a104filter = ch1data;
load experimenta108.mat
a108filter = ch1data;
load experimenta111.mat
a111filter = ch1data;
clear ch1data
time = linspace(0,20,20000);
%original
figure
plot(time,a111filter)
hold on;
%plot(time, data2array)
SF = 1000;
order = 2;
%filter at 20
cutoff_freq = 20/500;
[b, a] = butter(order, cutoff_freq, "high");
y = filtfilt(b, a, a111filter);
plot(time,y)
%filter at 250
cutoff_freq2 = 250/500;
[b2, a2] = butter(order, cutoff_freq2, "low");
y2 = filtfilt(b2, a2, a111filter);
plot(time,y2)
y_bandpass = filtfilt(b2, a2, y);
plot(time,y_bandpass)
legend('original', 'filter@20', "filter@250",'bandpassfilter')

figure
subplot(4,1,1);
plot(time, a111filter);
title('Original a111 data')
ylabel('Amplitude')
subplot(4,1,2);
plot(time, y);
title('High pass a111 data')
ylabel('Amplitude')
subplot(4,1,3);
plot(time,y2);
title('Low pass a111 data')
ylabel('Amplitude')
subplot(4,1,4)
plot(time,y_bandpass);
title('band pass a111 data')
ylabel('Amplitude')
xlabel('Time(s)')
%%
yabsfilt = abs(y_bandpass);
yabs = abs(a111filter);
figure
plot(time,yabsfilt)
hold on 
plot(time, yabs)
title('Rectified a111 data')
xlabel('Time(s)')
ylabel('Amplitude')
legend('Rectified Filtered a111 Data', 'Rectified Original a111 Data')
%% Envelope and Analysis
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(yabs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection

% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);

% Plot the original signal
figure
plot(time, yabs);
hold on; % Allows plotting multiple data sets on the same figure

% Overlay the envelope on the same plot
plot(time, Env); % Plots the upper envelope with a thicker line
plot(time, Envmean)
% xlim([1 7])
% ylim([-0.5 3])

hold off;
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Rectified a111 Signal with Envelope');
legend('Original a111 Rectified Signal', 'Envelope', 'Mean Envelope');
%%
time = linspace(0,20,20000);
yabs = abs(a104filter);
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(yabs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection

% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);

% Plot the original signal
figure
plot(time, yabs);
hold on; % Allows plotting multiple data sets on the same figure

% Overlay the envelope on the same plot
plot(time, Env); % Plots the upper envelope with a thicker line
plot(time, Envmean)
% xlim([1 7])
% ylim([-0.5 3])

hold off;
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Rectified a104 Signal with Envelope');
legend('Original a104 Rectified Signal', 'Envelope', 'Mean Envelope');
%%
yabs = abs(a108filter);
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(yabs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection

% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);

% Plot the original signal
figure
plot(time, yabs);
hold on; % Allows plotting multiple data sets on the same figure

% Overlay the envelope on the same plot
plot(time, Env); % Plots the upper envelope with a thicker line
plot(time, Envmean)
xlim([0 7])

hold off;
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Rectified a108 Signal with Envelope');
legend('Original a108 Rectified Signal', 'Envelope', 'Mean Envelope');
%% Same thing for Experiment B
% loading up new data:
load experimentb111-bicep.mat
bbicep = ch1data;
load experimentb111-tricep.mat
btricep = ch2data;
clear ch1data
clear ch2data
%original
%%figure
plot(time,bbicep)
hold on;
SF = 1000;
order = 2;
%filter at 20 bicep
cutoff_freq = 20/500;
[b, a] = butter(order, cutoff_freq, "high");
bbicepfilt20 = filtfilt(b, a, bbicep);
plot(time,bbicepfilt20)
%filter at 250 bicep
cutoff_freq2 = 250/500;
[b2, a2] = butter(order, cutoff_freq2, "low");
bbicepfilt250 = filtfilt(b2, a2, bbicep);
plot(time,bbicepfilt250)
bbicep_bandpass = filtfilt(b2, a2, bbicepfilt20);
plot(time,bbicep_bandpass)
%filter at 20 tricep
cutoff_freq = 20/500;
[b, a] = butter(order, cutoff_freq, "high");
btricepfilt20 = filtfilt(b, a, btricep);
plot(time,btricepfilt20)
%filter at 250 tricep
cutoff_freq2 = 250/500;
[b2, a2] = butter(order, cutoff_freq2, "low");
btricepfilt250 = filtfilt(b2, a2, btricep);
plot(time,btricepfilt250)
btricep_bandpass = filtfilt(b2, a2, btricepfilt20);
plot(time,btricep_bandpass)
legend('original bicep', 'filter@20 bicep', "filter@250 bicep",'bandpassfilter bicep', ...
    'original tricep', 'filter@20 tricep', "filter@250 tricep",'bandpassfilter tricep' )
% biceps grafs lol
figure
subplot(4,1,1);
plot(time, bbicep);
title('Original bicep data')
ylabel('Amplitude')
subplot(4,1,2);
plot(time, bbicepfilt20);
title('High pass bicep data')
ylabel('Amplitude')
subplot(4,1,3);
plot(time,bbicepfilt250);
title('Low pass bicep data')
ylabel('Amplitude')
subplot(4,1,4)
plot(time,bbicep_bandpass);
title('band pass bicep data')
ylabel('Amplitude')
xlabel('Time(s)')
% triceps grafs lol
figure
subplot(4,1,1);
plot(time, btricep);
title('Original tricep data')
ylabel('Amplitude')
subplot(4,1,2);
plot(time, btricepfilt20);
title('High pass tricep data')
ylabel('Amplitude')
subplot(4,1,3);
plot(time,btricepfilt250);
title('Low pass tricep data')
ylabel('Amplitude')
subplot(4,1,4)
plot(time,btricep_bandpass);
title('band pass tricep data')
ylabel('Amplitude')
xlabel('Time(s)')
% plotting rectified data
bbicep_absfilt = abs(bbicep_bandpass);
bbicep_abs = abs(bbicep);
figure
plot(time,bbicep_absfilt)
hold on 
plot(time, bbicep_abs)
title('Rectified biceps/triceps data')
xlabel('Time(s)')
ylabel('Amplitude')
btricep_absfilt = abs(btricep_bandpass);
btricep_abs = abs(btricep);
plot(time,btricep_absfilt) 
plot(time, btricep_abs)
%title('Rectified triceps data')
xlabel('Time(s)')
ylabel('Amplitude')
legend('Rectified Filtered Biceps Data', 'Rectified original biceps Data', 'Rectified Filtered Triceps Data', 'Rectified original triceps Data')
hold off
% Envelope and Analysis

% biceps upper envelope:
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(bbicep_abs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection
% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);
% Plot the original biceps signal
figure
plot(time, bbicep_abs);
hold on; 
plot(time, Env)
plot(time, Envmean)
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Rectified Biceps/Triceps Signal with Envelope');

% triceps upper envelope:
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(btricep_abs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection
% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);
% Plot the original triceps signal
plot(time, btricep_abs); 
plot(time, Env)
plot(time, Envmean)
legend('Original Biceps Rectified Signal', 'Envelope', 'Mean Envelope', ...
    'Original Triceps Rectified Signal', 'Envelope', 'Mean Envelope');
hold off;
%% Same thing for Experiment C
% loading up new data:
load experimentc111-bicep.mat
cbicep = ch1data;
load experimentc111-tricep.mat
ctricep = ch2data;
clear ch1data
clear ch2data
%original
%%figure
plot(time,cbicep)
hold on;
SF = 1000;
order = 2;
%filter at 20 bicep
cutoff_freq = 20/500;
[b, a] = butter(order, cutoff_freq, "high");
cbicepfilt20 = filtfilt(b, a, cbicep);
plot(time,cbicepfilt20)
%filter at 250 bicep
cutoff_freq2 = 250/500;
[b2, a2] = butter(order, cutoff_freq2, "low");
cbicepfilt250 = filtfilt(b2, a2, cbicep);
plot(time,cbicepfilt250)
cbicep_bandpass = filtfilt(b2, a2, cbicepfilt20);
plot(time,cbicep_bandpass)
%filter at 20 tricep
cutoff_freq = 20/500;
[b, a] = butter(order, cutoff_freq, "high");
ctricepfilt20 = filtfilt(b, a, ctricep);
plot(time,ctricepfilt20)
%filter at 250 tricep
cutoff_freq2 = 250/500;
[b2, a2] = butter(order, cutoff_freq2, "low");
ctricepfilt250 = filtfilt(b2, a2, ctricep);
plot(time,ctricepfilt250)
ctricep_bandpass = filtfilt(b2, a2, ctricepfilt20);
plot(time,ctricep_bandpass)
legend('original bicep', 'filter@20 bicep', "filter@250 bicep",'bandpassfilter bicep', ...
    'original tricep', 'filter@20 tricep', "filter@250 tricep",'bandpassfilter tricep' )
% biceps grafs lol
figure
subplot(4,1,1);
plot(time, cbicep);
title('Original bicep data')
ylabel('Amplitude')
subplot(4,1,2);
plot(time, cbicepfilt20);
title('High pass bicep data')
ylabel('Amplitude')
subplot(4,1,3);
plot(time,cbicepfilt250);
title('Low pass bicep data')
ylabel('Amplitude')
subplot(4,1,4)
plot(time,cbicep_bandpass);
title('band pass bicep data')
ylabel('Amplitude')
xlabel('Time(s)')
% triceps grafs lol
figure
subplot(4,1,1);
plot(time, ctricep);
title('Original tricep data')
ylabel('Amplitude')
subplot(4,1,2);
plot(time, ctricepfilt20);
title('High pass tricep data')
ylabel('Amplitude')
subplot(4,1,3);
plot(time,ctricepfilt250);
title('Low pass tricep data')
ylabel('Amplitude')
subplot(4,1,4)
plot(time,ctricep_bandpass);
title('band pass tricep data')
ylabel('Amplitude')
xlabel('Time(s)')
% plotting rectified data
cbicep_absfilt = abs(cbicep_bandpass);
cbicep_abs = abs(cbicep);
figure
plot(time,cbicep_absfilt)
hold on 
plot(time, cbicep_abs)
title('Rectified biceps/triceps data')
xlabel('Time(s)')
ylabel('Amplitude')
ctricep_absfilt = abs(ctricep_bandpass);
ctricep_abs = abs(ctricep);
plot(time,ctricep_absfilt) 
plot(time, ctricep_abs)
%title('Rectified triceps data')
xlabel('Time(s)')
ylabel('Amplitude')
legend('Rectified Filtered Biceps Data', 'Rectified original biceps Data', 'Rectified Filtered Triceps Data', 'Rectified original triceps Data')
hold off
% Envelope and Analysis

% biceps upper envelope:
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(cbicep_abs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection
% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);
% Plot the original biceps signal
figure
plot(time, cbicep_abs);
hold on; 
plot(time, Env)
plot(time, Envmean)
xlabel('Time (seconds)');
ylabel('Amplitude');
title('Rectified Biceps/Triceps Signal with Envelope');

% triceps upper envelope:
% Use the 'envelope' function to get the upper envelope of the signal
[Env, lowerEnv] = envelope(ctricep_abs, round(1000*0.01), 'peak'); % 'round(Fs*0.01)' is an example window length for peak detection
% Calculate the moving mean for the upper envelope
Envmean = movmean(abs(Env), 50);
% Plot the original triceps signal
plot(time, ctricep_abs); 
plot(time, Env)
plot(time, Envmean)
legend('Original Biceps Rectified Signal', 'Envelope', 'Mean Envelope', ...
    'Original Triceps Rectified Signal', 'Envelope', 'Mean Envelope');
hold off;