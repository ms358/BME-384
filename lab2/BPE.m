clear all;close all
ver = computer;
if strcmp(ver,'PCWIN64')
    dll = 'C:\Program Files (x86)\BIOPAC Systems, Inc\BIOPAC Hardware API 2.2 Education\x64\mpdev.dll';
else
    dll = 'C:\Program Files (x86)\BIOPAC Systems, Inc\BIOPAC Hardware API 2.2 Education\Win32\mpdev.dll';
end
dothdir = 'C:\Program Files (x86)\BIOPAC Systems, Inc\BIOPAC Hardware API 2.2 Education\mpdev.h';
mptype = 103; % MP36
mpmethod = 10; % USB
sn = 'auto';
libname = 'mpdev';
warning off MATLAB:loadlibrary:enumexists;
loadlibrary(dll,dothdir);

sample_rate = 1000; % Sample Rate
n=input('how many channels are you acquiring? \n');% Channel selection
channel_mask = [0 0 0 0]; 
for i=1:n
    channel_mask(1,i) = 1; %   setting channels which are used
end
channels = {'a113' 'a113' 'a113' 'a113'}; % Channel presets
retval = calllib(libname,'connectMPDev',mptype,mpmethod,sn); % connect to device
retval = calllib(libname, 'setSampleRate', double(1000/sample_rate)); % Set Sample Rate
retval = calllib(libname, 'setAcqChannels', channel_mask); % Set Acquisition Channels
retval = calllib(libname, 'loadXMLPresetFile', 'C:\Program Files (x86)\BIOPAC Systems, Inc\BIOPAC Hardware API 2.2 Education\PresetFiles\channelpresets.xml'); % load preset file
k = find(channel_mask==1);
for i=1:length(k)
    assignin('base',strcat('ch',num2str(k(i)),'data'),[]);
    retval = calllib(libname, 'configChannelByPresetID', k(i)-1, channels{k(i)}); % Configure the channels by preset ID
end
retval = calllib(libname, 'startMPAcqDaemon'); % Create virtual server
retval = calllib(libname, 'startAcquisition'); % Start acquisition

T = input('what is the duration of the trial? \n'); % collect total of T secons data
t = 0.1; % collect t seconds of data per iteration
numValuesToRead = t*sample_rate*sum(channel_mask); % data points per iteration
remaining = T*sample_rate*sum(channel_mask); % remaining data points to acquire
tbuff(1:numValuesToRead) = double(0); % initialize the correct amount of data
numRead = 0;
figure
   
while remaining >0
    [retval, tbuff, numRead]  = calllib(libname, 'receiveMPData',tbuff, numValuesToRead, numRead); % get data
    pause(.05); clf; hold on
    remaining = remaining - numRead;
    for i=1:sum(channel_mask)
        assignin('base',strcat('ch',num2str(k(i)),'data'),[eval(strcat('ch',num2str(k(i)),'data')) tbuff(i:sum(channel_mask):length(tbuff))]);
        plot(eval(strcat('ch',num2str(k(i)),'data'))); xlabel('Milliseconds');ylabel('Volts');
    end
    hold off
    
end

for i=1:n
save(strcat('experiment',num2str(k(i))),(strcat('ch',num2str(k(i)),'data')));
end
retval = calllib(libname, 'stopAcquisition') % stop acquisition
calllib(libname, 'disconnectMPDev'); % disconnect device
unloadlibrary(libname) % unload library