
%% Create AlphaBot Line Following Control - Simulink Model
% Run this script in MATLAB with Simulink installed.
% It creates and saves: AlphaBot_Line_Following_Control.slx

model = 'AlphaBot_Line_Following_Control';

if bdIsLoaded(model)
    close_system(model, 0);
end

new_system(model);
open_system(model);

set_param(model, 'Solver', 'ode45');
set_param(model, 'StopTime', '10');

%% Positions
pos_in_pixy      = [60 170 110 200];
pos_target_calc  = [180 135 330 230];
pos_sum_error    = [430 165 465 205];
pos_frame_center = [230 340 330 390];
pos_pd           = [570 145 670 225];
pos_sum_steer    = [770 165 805 205];
pos_gyro         = [500 345 550 375];
pos_ky           = [640 325 730 395];
pos_motor        = [890 135 1010 235];
pos_scope        = [1080 165 1130 205];
pos_sensor_gain  = [500 520 590 590];

%% Blocks

add_block('simulink/Sources/In1', [model '/Pixy2_Vector_Input'], ...
    'Position', pos_in_pixy);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Target X Calculation'], ...
    'Position', pos_target_calc);

subsys = [model '/Target X Calculation'];

try
    delete_line(subsys, 'In1/1', 'Out1/1');
catch
end

set_param([subsys '/In1'], 'Name', 'pixyVector');
set_param([subsys '/Out1'], 'Name', 'targetX');

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [subsys '/TargetX_Function'], ...
    'Position', [120 70 280 150]);

rt = sfroot;
chart = rt.find('-isa','Stateflow.EMChart','Path',[subsys '/TargetX_Function']);
chart.Script = sprintf([ ...
    'function targetX = fcn(pixyVector)\n' ...
    '%% pixyVector = [x0 y0 x1 y1]\n' ...
    'x0 = pixyVector(1);\n' ...
    'x1 = pixyVector(3);\n' ...
    'targetX = (x0 + x1)/2;\n' ...
    'end\n']);

add_line(subsys, 'pixyVector/1', 'TargetX_Function/1');
add_line(subsys, 'TargetX_Function/1', 'targetX/1');

add_block('simulink/Math Operations/Sum', [model '/Error Calculation'], ...
    'Inputs', '+-', ...
    'Position', pos_sum_error);

add_block('simulink/Sources/Constant', [model '/Frame_Center'], ...
    'Value', '39', ...
    'Position', pos_frame_center);

add_block('simulink/Continuous/PID Controller', [model '/PD Controller'], ...
    'Position', pos_pd);

set_param([model '/PD Controller'], ...
    'Controller', 'PD', ...
    'P', '6.8', ...
    'D', '4.0');

add_block('simulink/Sources/In1', [model '/Gyro_YawRate'], ...
    'Position', pos_gyro);

add_block('simulink/Math Operations/Gain', [model '/Ky_Gain'], ...
    'Gain', '0.06', ...
    'Position', pos_ky);

add_block('simulink/Math Operations/Sum', [model '/Steering Command'], ...
    'Inputs', '++', ...
    'Position', pos_sum_steer);

add_block('simulink/Continuous/Transfer Fcn', [model '/Motor System / AlphaBot Dynamics'], ...
    'Numerator', '[1]', ...
    'Denominator', '[1 1]', ...
    'Position', pos_motor);

add_block('simulink/Sinks/Scope', [model '/Scope_Viewer'], ...
    'Position', pos_scope);

add_block('simulink/Math Operations/Gain', [model '/Pixy2 Sensor Gain'], ...
    'Gain', '1', ...
    'Position', pos_sensor_gain);

%% Connections
add_line(model, 'Pixy2_Vector_Input/1', 'Target X Calculation/1', 'autorouting', 'on');
add_line(model, 'Target X Calculation/1', 'Error Calculation/1', 'autorouting', 'on');
add_line(model, 'Frame_Center/1', 'Error Calculation/2', 'autorouting', 'on');
add_line(model, 'Error Calculation/1', 'PD Controller/1', 'autorouting', 'on');
add_line(model, 'PD Controller/1', 'Steering Command/1', 'autorouting', 'on');
add_line(model, 'Steering Command/1', 'Motor System / AlphaBot Dynamics/1', 'autorouting', 'on');
add_line(model, 'Motor System / AlphaBot Dynamics/1', 'Scope_Viewer/1', 'autorouting', 'on');

add_line(model, 'Gyro_YawRate/1', 'Ky_Gain/1', 'autorouting', 'on');
add_line(model, 'Ky_Gain/1', 'Steering Command/2', 'autorouting', 'on');

add_line(model, 'Motor System / AlphaBot Dynamics/1', 'Pixy2 Sensor Gain/1', 'autorouting', 'on');
add_line(model, 'Pixy2 Sensor Gain/1', 'Pixy2_Vector_Input/1', 'autorouting', 'on');

%% Annotations
Simulink.Annotation(model, 'AlphaBot Line Following Control - Simulink Model');
Simulink.Annotation(model, 'Pixy2 vector input: [x0 y0 x1 y1]');
Simulink.Annotation(model, 'targetX = (x0 + x1)/2');
Simulink.Annotation(model, 'Error: e = targetX - frameCenter');
Simulink.Annotation(model, 'PD: P = 6.8, D = 4.0');
Simulink.Annotation(model, 'Gyro correction: Ky = 0.06');
Simulink.Annotation(model, 'Frame center from Arduino code = 39');

set_param(model, 'ZoomFactor', 'FitSystem');
save_system(model, [model '.slx']);

disp('Created and saved: AlphaBot_Line_Following_Control.slx');
