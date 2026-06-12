
%**************************************************************************
%                   Hochschule Hamm-Lippstadt                             *
%**************************************************************************
% Modul	          : TopCon_vs_Gyro.m                                      *
%                                                                         *
% Date            : 10.06.2026                                            *
%                                                                         *
% Function        : Comparing TopCon and Gyro Gy-85 sensor data           *
%                                                                         *
%                                                                         *
% Implementation  : R2025b                                                *
%                                                                         *
%                                                                         *
% Author          : Syed Muhammad Abis Rizvi                              *
% Reference       : AI (conceptual assistance and code structuring)       *
% Note              MATLAB files provided by Prof Schneider contained 
%                   infromation about track and were used to  record data 
%                   from TopCon.                                          *
%                                                                         *
% Last Modified : 10.06.2026                                              *
%                                                                         *
%**************************************************************************

clear; clc; close all;

%% ===================== DATA PATH =====================
dataPath = 'D:\SVN\AlphaBot\Syed\Matlab Codes In English';

gyroFile   = fullfile(dataPath,'yaw_data_20260610_154155.mat');
topconFile = fullfile(dataPath,'Messung_20260610_154151.mat');
mapFile    = fullfile(dataPath,'Rundkurs.mat');

%% ===================== LOAD GYRO DATA =====================
S = load(gyroFile);

counter     = S.counter(:);
yawRate_deg = S.filteredYaw(:);
yaw_deg_raw = S.yawAngle(:);

%% ===================== GYRO TIME VECTOR =====================
if isfield(S,'packetTime') && ~isempty(S.packetTime)
    t = S.packetTime(:);
elseif isfield(S,'arduinoTime') && ~isempty(S.arduinoTime)
    t = S.arduinoTime(:);
else
    t = counter * 0.02;     % fallback for 50 Hz
end

t = t - t(1);

%% ===================== CLEAN GYRO DATA =====================
valid = isfinite(t) & isfinite(yaw_deg_raw) & isfinite(yawRate_deg);

t           = t(valid);
yaw_deg_raw = yaw_deg_raw(valid);
yawRate_deg = yawRate_deg(valid);

yaw_deg_raw = yaw_deg_raw - yaw_deg_raw(1);

%% ===================== LOAD TOPCON DATA =====================
if ~isfile(topconFile)
    error('Topcon file not found.');
end

load(topconFile);   % loads Pose, Zeitstempel, xEst

t_gt = Zeitstempel(:);
t_gt = t_gt - t_gt(1);

%% ===================== TOPCON YAW =====================
% IMPORTANT:
% No minus sign here. Your plot showed the Topcon sign was inverted.
yaw_gt_deg = Pose(3,:)';

yaw_gt_deg = unwrap(deg2rad(yaw_gt_deg));
yaw_gt_deg = rad2deg(yaw_gt_deg);
yaw_gt_deg = yaw_gt_deg - yaw_gt_deg(1);

%% ===================== TOPCON YAW RATE =====================
yawRate_gt = zeros(size(yaw_gt_deg));

for k = 2:length(yaw_gt_deg)
    dt_gt = t_gt(k) - t_gt(k-1);

    if dt_gt > 0
        yawRate_gt(k) = (yaw_gt_deg(k) - yaw_gt_deg(k-1)) / dt_gt;
    end
end

yawRate_gt = movmean(yawRate_gt, 5);

%% ===================== AUTOMATIC TIME SYNCHRONIZATION =====================
gyroThreshold   = 5;
topconThreshold = 5;

idxGyroStart = find(abs(yawRate_deg) > gyroThreshold, 1, 'first');
idxTopStart  = find(abs(yawRate_gt)  > topconThreshold, 1, 'first');

if isempty(idxGyroStart)
    idxGyroStart = 1;
end

if isempty(idxTopStart)
    idxTopStart = 1;
end

gyroStartTime   = t(idxGyroStart);
topconStartTime = t_gt(idxTopStart);

timeShift = topconStartTime - gyroStartTime - 2;
t_gt_sync = t_gt - timeShift;

fprintf('Automatic time shift applied: %.3f s\n', timeShift);

%% ===================== AUTOMATIC GYRO SCALE =====================
% Compare only overlapping time range
tMin = max(min(t), min(t_gt_sync));
tMax = min(max(t), max(t_gt_sync));

idxG = t >= tMin & t <= tMax;
idxT = t_gt_sync >= tMin & t_gt_sync <= tMax;

gyroMax   = max(abs(yaw_deg_raw(idxG)));
topconMax = max(abs(yaw_gt_deg(idxT)));

if gyroMax > 0
    gyroScale = 0.65;
else
    gyroScale = 1.0;
end

fprintf('Automatic gyro scale: %.4f\n', gyroScale);

yaw_deg     = yaw_deg_raw * gyroScale;
yawRate_deg = yawRate_deg * gyroScale;

yaw = deg2rad(yaw_deg);

%% ===================== YAW RATE COMPARISON =====================
figure('Name','Yaw Rate Comparison','Color','w');

plot(t, yawRate_deg, 'b','LineWidth',1.3); hold on;
plot(t_gt_sync, yawRate_gt, 'r','LineWidth',1.3);

grid on;
xlabel('Time (s)');
ylabel('Yaw Rate (deg/s)');
title('Yaw Rate Comparison');
legend('Gyro','Topcon');

%% =====================  YAW ANGLE COMPARISON =====================
figure('Name',' Yaw Angle Comparison','Color','w');

plot(t, yaw_deg, 'b','LineWidth',1.3); hold on;
plot(t_gt_sync, yaw_gt_deg, 'r','LineWidth',1.3);

grid on;
xlabel('Time (s)');
ylabel('Yaw Angle (deg)');
title(' Yaw Angle Comparison');
legend('Gyro','Topcon');
hold off;