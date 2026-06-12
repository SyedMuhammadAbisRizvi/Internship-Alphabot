
%**************************************************************************
%                   Hochschule Hamm-Lippstadt                             *
%**************************************************************************
% Modul	          : TopCon_vs_Gyro.m                                      *
%                                                                         *
% Date            : 10.06.2026                                            *
%                                                                         *
% Function        :  Getting Gyro Gy-85 sensor data via WLAN              *
%                     YawRate and YawAngle                                *
%                                                                         *
% Implementation  : R2025b                                                *
%                                                                         *
%                                                                         *
% Author          : Syed Muhammad Abis Rizvi                              *
% Reference       : AI (conceptual assistance and code structuring)       *
% Note                                                                    *
%                                                                         *
% Last Modified   : 10.06.2026                                            *
%                                                                         *
%**************************************************************************
clc;
clear;
close all;

%% Connect to Arduino TCP Server
t = tcpclient("192.168.1.23", 5000);

%% Data storage
counter     = [];
yawRate     = [];
filteredYaw = [];
yawAngle    = [];
packetTime  = [];

alpha = 0.1; %% Sets the smoothing factor%%

%% Timers
lastDataTime = tic;
startTime    = tic;

%% Autosave settings
autosaveFile = 'yaw_data_autosave.mat';
autosaveEverySamples = 100;

%% Create figure
hFig = figure;
disp("Starting data acquisition...");

%% Main loop
while true

    try
        %% Stop and final save if figure is closed
        if ~isvalid(hFig)
            saveYawData(counter, yawRate, filteredYaw, yawAngle, packetTime, "final");
            break;
        end

        %% Check for incoming data
        if t.NumBytesAvailable > 0

            %% Read line from Arduino
            data = readline(t);
            currentPacketTime = toc(startTime);

            disp("Received:")
            disp(data)

            %% Reset timeout timer
            lastDataTime = tic;

            %% Parse data
            vals = str2double(split(strtrim(data), ","));

            disp("Parsed values:")
            disp(vals)

            %% Arduino sends: yawRate, yawAngle, counter
            if numel(vals) ~= 3 || any(isnan(vals))
                disp("Invalid packet:")
                disp(data)
                continue;
            end

            rawYaw = vals(1);
            angle  = vals(2);
            count  = vals(3);

            %% Low-pass filter in MATLAB
            if isempty(filteredYaw)
                filtYaw = rawYaw;
            else
                filtYaw = (1 - alpha) * filteredYaw(end) + alpha * rawYaw;
            end

            %% Store data
            yawRate(end+1)     = rawYaw;
            filteredYaw(end+1) = filtYaw;
            yawAngle(end+1)    = angle;
            counter(end+1)     = count;
            packetTime(end+1)  = currentPacketTime;

            %% Autosave while running
            if mod(length(counter), autosaveEverySamples) == 0
                save(autosaveFile, ...
                    'counter', ...
                    'yawRate', ...
                    'filteredYaw', ...
                    'yawAngle', ...
                    'packetTime');

                disp(['Autosaved to ', autosaveFile]);
            end

            %% Plot using MATLAB receive timestamp
            plotTime = packetTime - packetTime(1);

            subplot(4,1,1)
            plot(plotTime, yawRate, 'b', 'LineWidth', 1.5)
            title('Raw Yaw Rate')
            ylabel('deg/s')
            grid on

            subplot(4,1,2)
            plot(plotTime, filteredYaw, 'g', 'LineWidth', 1.5)
            title('Filtered Yaw Rate')
            ylabel('deg/s')
            grid on

            subplot(4,1,3)
            plot(plotTime, yawAngle, 'r', 'LineWidth', 1.5)
            title('Yaw Angle')
            ylabel('deg')
            grid on

            subplot(4,1,4)
            plot(plotTime, counter, 'k', 'LineWidth', 1.5)
            title('Counter')
            ylabel('Count')
            xlabel('Time (s)')
            grid on

            drawnow;

        else
            pause(0.01);

            %% Stop and save only if no data for 20 seconds
            if toc(lastDataTime) > 20
                disp("Robot disconnected or no data received.");
                saveYawData(counter, yawRate, filteredYaw, yawAngle, packetTime, "final");
                break;
            end
        end

    catch ME
        disp("Connection error occurred:");
        disp(ME.message);

        saveYawData(counter, yawRate, filteredYaw, yawAngle, packetTime, "final");
        break;
    end
end

%% Cleanup TCP connection
clear t;

disp("Script finished.");

%% Function to save yaw data
function saveYawData(counter, yawRate, filteredYaw, yawAngle, packetTime, mode)

    if isempty(counter)
        disp("No valid data received. Nothing useful to save.");
        return;
    end

    timestamp = datestr(now, 'yyyymmdd_HHMMSS');

    if mode == "final"
        filename = sprintf('yaw_data_%s.mat', timestamp);
    else
        filename = 'yaw_data_autosave.mat';
    end

    save(filename, ...
        'counter', ...
        'yawRate', ...
        'filteredYaw', ...
        'yawAngle', ...
        'packetTime');

    disp(['Data saved as: ', filename]);

end