%Real-time plot of Arduino CSV: time_ms,kalAngleX,kalAngleY,compAngleX,compAngleY
clearvars; close all; clc;

% ------- USER SETTINGS -------
Port = 'COM5';         % change if needed
baudRate = 115200;
XLim = 20;             % seconds shown on x-axis (scroll window)
initialYlim = [-180 180]; % initial Y limits (angles in degrees)
% -----------------------------

% Create serial object
s = serialport(Port, baudRate);

% Configure terminator and flush
configureTerminator(s, "LF");
flush(s);

% Figure + animated lines
fig = figure('Name','Live: Kalman & Complementary Angles','NumberTitle','off');
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Angle (deg)');
title(ax, 'Real-time: kalAngleX, kalAngleY, compAngleX, compAngleY');

% Four animated lines
hKalX  = animatedline('Color',[0 0.447 0.741],'LineWidth',1.5,'DisplayName','kalAngleX');   % blue
hKalY  = animatedline('Color',[0.85 0.325 0.098],'LineWidth',1.5,'DisplayName','kalAngleY'); % red
hCompX = animatedline('Color',[0.466 0.674 0.188],'LineWidth',1.5,'DisplayName','compAngleX'); % green
hCompY = animatedline('Color',[0.494 0.184 0.556],'LineWidth',1.5,'DisplayName','compAngleY'); % purple

legend(ax, 'show', 'Location','best');
ax.YLim = initialYlim;
ax.XLim = [0 XLim];

% Stop flag and keypress callback
setappdata(fig, 'stopNow', 0);
set(fig, 'KeyPressFcn', @(src,ev) keypressCallback(src,ev));

% Storage for data: columns = time_s, kalX, kalY, compX, compY
allData = zeros(0,5);

% Data loop
dataIndex = 0;
try
    disp('Starting data read. Press ''q'' in the figure window to stop.');
    while ~getappdata(fig,'stopNow')
        if s.NumBytesAvailable > 0
            raw = readline(s);
            raw = strtrim(raw);

            if contains(raw, 'time_ms','IgnoreCase',true)
                continue;
            end

            parts = strsplit(raw, ',');
            if numel(parts) ~= 5
                fprintf('Warning: expected 5 fields but got %d: %s\n', numel(parts), raw);
                continue;
            end

            t_ms   = str2double(parts{1});
            kalX   = str2double(parts{2});
            kalY   = str2double(parts{3});
            compX  = str2double(parts{4});
            compY  = str2double(parts{5});

            if any(isnan([t_ms, kalX, kalY, compX, compY]))
                fprintf('Warning: NaN detected in parsed data: %s\n', raw);
                continue;
            end

            t_s = double(t_ms)/1000;

            allData(end+1, :) = [t_s, kalX, kalY, compX, compY]; %#ok<SAGROW>
            dataIndex = dataIndex + 1;

            addpoints(hKalX, t_s, kalX);
            addpoints(hKalY, t_s, kalY);
            addpoints(hCompX, t_s, compX);
            addpoints(hCompY, t_s, compY);

            if t_s < XLim
                ax.XLim = [0 XLim];
            else
                ax.XLim = [t_s - XLim, t_s];
            end

            currY = ax.YLim;
            margin = 5;
            minVal = min([kalX, kalY, compX, compY]);
            maxVal = max([kalX, kalY, compX, compY]);
            if minVal - margin < currY(1) || maxVal + margin > currY(2)
                ax.YLim = [min(currY(1), minVal - margin), max(currY(2), maxVal + margin)];
            end

            drawnow limitrate;

            if mod(dataIndex, 200) == 0
                flush(s);
            end
        else
            pause(0.005);
        end
    end
catch ME
    fprintf('Error: %s\n', ME.message);
end

% Clean up serial object
try
    clear s;
catch
end

disp('Stopped reading data.');

% --- Kalman Filter Postprocessing ---
disp('Applying Kalman filter postprocessing...');

Q_angle = 0.001;
Q_bias = 0.003;
R_measure = 0.03;

angleX = allData(1,4);
biasX = 0;
P_X = [0 0; 0 0];

angleY = allData(1,5);
biasY = 0;
P_Y = [0 0; 0 0];

kalmanPostX = zeros(size(allData,1),1);
kalmanPostY = zeros(size(allData,1),1);

kalmanPostX(1) = angleX;
kalmanPostY(1) = angleY;

for k = 2:size(allData,1)
    dt = allData(k,1) - allData(k-1,1);
    if dt <= 0, dt = 1e-3; end

    rateX = (allData(k,4) - allData(k-1,4)) / dt;
    rateY = (allData(k,5) - allData(k-1,5)) / dt;

    % Kalman X
    angleX = angleX + dt * (rateX - biasX);
    P_X = P_X + dt * ([Q_angle -P_X(1,2); -P_X(2,1) Q_bias]);

    S = P_X(1,1) + R_measure;
    K = P_X(:,1) / S;
    y = allData(k,4) - angleX;
    angleX = angleX + K(1) * y;
    biasX  = biasX  + K(2) * y;
    P_X = P_X - K * P_X(1,:);

    kalmanPostX(k) = angleX;

    % Kalman Y
    angleY = angleY + dt * (rateY - biasY);
    P_Y = P_Y + dt * ([Q_angle -P_Y(1,2); -P_Y(2,1) Q_bias]);

    S = P_Y(1,1) + R_measure;
    K = P_Y(:,1) / S;
    y = allData(k,5) - angleY;
    angleY = angleY + K(1) * y;
    biasY  = biasY  + K(2) * y;
    P_Y = P_Y - K * P_Y(1,:);

    kalmanPostY(k) = angleY;
end

% --- Plot postprocessed Kalman results ---
figure('Name','Postprocessed Kalman Filter','NumberTitle','off');
plot(allData(:,1), allData(:,4), 'g--', 'DisplayName','compAngleX'); hold on;
plot(allData(:,1), kalmanPostX, 'b', 'DisplayName','kalmanPostX');
plot(allData(:,1), allData(:,5), 'm--', 'DisplayName','compAngleY');
plot(allData(:,1), kalmanPostY, 'r', 'DisplayName','kalmanPostY');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Kalman Filter Postprocessing vs Complementary Filter');
legend('Location','best');
grid on;

% --- Save data prompt ---
if ~isempty(allData)
    fprintf('\n=== Data Collection Summary ===\n');
    fprintf('Total samples collected: %d\n', size(allData,1));
    fprintf('Time range: %.3f to %.3f s\n', allData(1,1), allData(end,1));
    fprintf('kalAngleX: %.3f to %.3f deg\n', min(allData(:,2)), max(allData(:,2)));

    saveChoice = input('Save data to file? (y/n): ', 's');
    if strcmpi(saveChoice,'y')
        T = array2table([allData, kalmanPostX, kalmanPostY], ...
            'VariableNames', {'time_s','kalAngleX','kalAngleY','compAngleX','compAngleY','kalmanPostX','kalmanPostY'});
        filename = sprintf('gimbal_data_%s.csv', datestr(now,'yyyy-mm-dd_HH-MM-SS'));
        writetable(T, filename);
        fprintf('Data saved to %s\n',
