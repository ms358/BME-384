function [] = funky(file1, file2, label1, label2, color1, color2, dataset)
figure
SF = 1000;
data1 = open(file1);% input data1 as string
data2 = open(file2);% input data2 as string
% loading up the data
data1 = data1.ch1data;
data2 = data2.ch2data;
% setting up the time array to match each data array
Time = linspace(0,data1(end)*SF,size(data1,2));
hold on
% filtering
[A, B] = butter(2,5/500, "low");
data1filtered = filtfilt(A,B,data1);
data2filtered = filtfilt(A,B,data2);
% plotting
yyaxis left
plot(Time,data2filtered, "Color", color1)
xlabel('Time (s)')
ylabel('Right Hand Force')
yyaxis right
plot(Time,data1filtered, "Color", color2)
ylabel('Left Hand Force')
legend(label1, label2)
hold off
% find the derivative
vel2 = diff(data2filtered)./diff(Time);
vel2 = [vel2,vel2(end)];
vel1 = diff(data1filtered)./diff(Time);
vel1 = [vel1,vel1(end)];
% filtering derivative
[A, B] = butter(4,5/500, "low");
vel2filtered = filtfilt(A, B, vel2);
vel1filtered = filtfilt(A, B, vel1);
figure
yyaxis left
plot(Time,vel2filtered, "Color", color1)
hold on
ylabel('Velocity For Right Hand')
yyaxis right
plot(Time,vel1filtered, "Color", color2)
ylabel('Velocity For Left Hand')
xlabel('Time (s)')
hold off
% **ONLY FOR CONDITIONS 1 AND 4** using xcorr to find the delay
if (dataset == 1 || dataset == 4)
    [correlation, lag] = xcorr(vel2,vel1, 200); 
    figure
    stem(lag, correlation)
    xlabel("Time Offset (milliseconds)")
    ylabel("Cross Correlation")
end