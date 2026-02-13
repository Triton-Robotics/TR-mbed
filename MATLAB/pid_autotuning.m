%% Load data from .txt
filename = 'sentry yaw speed ff.txt';
response_raw = readmatrix(filename);
% Parameters
step_amp = 8191;
fall_amp = 2*step_amp;
dt_ms = 15; % set by main.cpp
%% Step Response
input = response_raw(:,1);
idx = input == step_amp;
response = response_raw(idx,2);
%% Impulse Response
% input = response_raw(:,1);
% idx = find(response_raw(:,1) == step_amp);
% % response = response_raw(idx,:);
% response = response_raw(idx-20:idx+20,2);
% % response = response(abs(response) - abs(2*mean(response)) < 0);
%% Wide pulse
% input = response_raw(:,1);
% pos_idx = input == step_amp;
% pos_edge_idx = find(pos_idx,1);
% 
% neg_idx = input == -step_amp;
% neg_edge_idx = find(neg_idx,1);
% pulse_width = (neg_edge_idx - pos_edge_idx) * dt_ms; % Width of first step
% 
% input = input(pos_edge_idx:end);
% response = response_raw(pos_edge_idx:end,2);
%% Arbitrary response
% input = response_raw(:,1);
% start_idx = find(input > 0,1);
% 
% input = input(start_idx:end);
% response = response_raw(start_idx:end,2);
%% Modify response if necessary
% Remove top 1% of values, read errors usually give values in the millions
outlier_idx = isoutlier(response,"percentiles",[0 99]); 
response = response(~outlier_idx);
input = input(~outlier_idx);

% For dealing with angle, compensate for wraparound
% add_idx = response < 6000; % Change const as needed
% response(add_idx) = response(add_idx) + 8192;

% For angle, start at 0 for first value
% first_val = response(1);
% response = response - first_val;

% Shorten signal
% len = 200;
% response = response(1:len);
% input = input(1:len);

% Flip signal
% response = -response;
% input = -input;
%% FF
% Parameters
degree = 2;

input = response_raw(:,1);
response = response_raw(:,2);

% Clean Data
outlier_idx = isoutlier(response,"percentiles",[0 99]); 
response = response(~outlier_idx);
input = input(~outlier_idx);
valid = ~isnan(input) & ~isnan(response);
input = input(valid);
response = response(valid);

% Set axis variables
y = response;
x = input;

% Linear fit: degree 1
p = polyfit(y, x, degree); 
y_fit = polyval(p, y);

% Plot
plot(y, x, 'o');             % original data
hold on;
plot(y, y_fit, '-r');        % best-fit line
legend('Data', 'Best Fit');
title('Linear Best Fit using polyfit');
xlabel('x'); ylabel('y');

%% Load idss
% load('tunedModels.mat')
%% Save idss after export
% S = whos;
% modelNames = {S(strcmp({S.class},'idss') | strcmp({S.class},'idproc')).name};
% save('tunedModels.mat', modelNames{:});