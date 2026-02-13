filename = 'response_readings/vel_20.txt';
response_raw = readmatrix(filename);

%%
input = response_raw(:,2);
idx = input == 20;
input = response_raw(idx,2);
response_20 = min(abs(response_raw(idx,1))); %in milli Nm convert

%%

ip = [5 10 15 20 25 30 35] * 0.1047;
op = [response_5, response_10, response_15, response_20, response_25, response_30, response_35];

% plot (ip, op);

coefficients = polyfit(ip, op, 1);
% Create a new x axis with exactly 1000 points (or whatever you want).
xFit = linspace(min(ip), max(op));
% Get the estimated yFit value for each of those 1000 new x locations.
yFit = polyval(coefficients , xFit);
% Plot everything.
plot(ip, op, 'b.', 'MarkerSize', 15); % Plot training data.
hold on; % Set hold on so the next plot does not blow away the one we just drew.
plot(xFit, yFit, 'r-', 'LineWidth', 2); % Plot fitted line.
grid on;