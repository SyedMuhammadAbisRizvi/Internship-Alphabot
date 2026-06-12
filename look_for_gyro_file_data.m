clear; clc; close all;

%% Load gyro file
S = load('yaw_data_20260610_154155.mat');

%% Extract data
yawRate = S.filteredYaw;
yawAngle = S.yawAngle;

%% Time axis
if isfield(S,'packetTime')
    t = S.packetTime;
elseif isfield(S,'arduinoTime')
    t = S.arduinoTime;
else
    t = S.counter * 0.02;   % fallback
end

t = t - t(1);

%% Plot yaw rate
figure;
plot(t, yawRate,'b','LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw Rate (deg/s)');
title('Gyro Yaw Rate');

%% Plot yaw angle
figure;
plot(t, yawAngle,'r','LineWidth',1.5);
grid on;
xlabel('Time (s)');
ylabel('Yaw Angle (deg)');
title('Gyro Yaw Angle');

%% Show duration
fprintf('Duration = %.2f seconds\n', t(end));
fprintf('Samples  = %d\n', length(t));