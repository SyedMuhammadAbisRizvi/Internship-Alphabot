%% Arduino Bluetooth Dashboard (Wireless via Sensor02)
clc; clear;

disp('Starting Arduino Bluetooth dashboard...');
disp('Make sure Sensor02 (HC-05/HC-06) is powered and paired.');

% Connect to HC-05 / Sensor02
try
    HC05 = bluetooth("Sensor02",1); % Channel 1 for SPP
    configureTerminator(HC05,"LF");  % Arduino sends lines ending with \n
    flush(HC05); pause(1);
    disp('Bluetooth connection established.');
catch
    error('Could not connect to Sensor02. Make sure it is paired and powered.');
end

% Live plot setup
figure('Name','Arduino Bluetooth Dashboard','NumberTitle','off');
h1 = animatedline('Color','r','DisplayName','Heading');
h2 = animatedline('Color','b','DisplayName','YawAngle');
h3 = animatedline('Color','g','DisplayName','YawRateGyro');
h4 = animatedline('Color','m','DisplayName','LineError');
h5 = animatedline('Color','c','DisplayName','LeftPower');
h6 = animatedline('Color','k','DisplayName','RightPower');
legend show; xlabel('Time (s)'); ylabel('Values'); grid on;

startTime = datetime('now');

disp('Receiving data...');

while ishandle(h1)
    try
        % Read one line from Arduino via Bluetooth
        line = readline(HC05);
        
        % Split by '|'
        tokens = split(line,"|");

        if length(tokens) < 6
            continue; % ignore incomplete lines
        end

        % Parse numeric values
        Heading       = str2double(extractAfter(tokens{1},":"));
        YawAngle      = str2double(extractAfter(tokens{2},":"));
        YawRateGyro   = str2double(extractAfter(tokens{3},":"));
        LineError     = str2double(extractAfter(tokens{4},":"));
        LeftPower     = str2double(extractAfter(tokens{5},":"));
        RightPower    = str2double(extractAfter(tokens{6},":"));

        % Current time for plotting
        t = seconds(datetime('now') - startTime);

        % Add points to animated lines
        addpoints(h1,t,Heading);
        addpoints(h2,t,YawAngle);
        addpoints(h3,t,YawRateGyro);
        addpoints(h4,t,LineError);
        addpoints(h5,t,LeftPower);
        addpoints(h6,t,RightPower);

        drawnow limitrate % update plot efficiently
    catch
        % Ignore read errors
        continue;
    end
end % <-- END of while loop