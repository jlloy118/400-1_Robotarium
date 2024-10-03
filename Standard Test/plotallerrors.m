close all;
clear all;

% % Set up the time range
% XieRussoNoBrakes;
% SilvaAdaptationClean;
% ActuatorDynamicsTest;
% save("starttime","start_time");
close all;
clear all;
ts = 0.033;
iterations = 4000; % Changed to 3000
start_time = 4000-3151+1;
t = 1:(iterations-start_time+1);
t = t*ts;

% Load the necessary data
silvanomodsupremum = load("supremerrorsilvanomods.mat", "supremum_reference_error");
silvanomodssupremum = silvanomodsupremum.supremum_reference_error;

xienomodssupremum = load("supremerrorxienomods.mat", "supremum_reference_error");
xienomodssupremum = xienomodssupremum.supremum_reference_error;

silvactnomodssupremum = load("silvactnomodssupremum.mat", "supremum_reference_error");
silvactnomodssupremum = silvactnomodssupremum.supremum_reference_error;

silvaactnomods = load("silvactnomodsspacing.mat", "max_spacing_error");
silvaactnomods = silvaactnomods.max_spacing_error;

silvanomodsspacing = load("silvanomodsspacing.mat", "max_spacing_error");
silvanomodsspacing = silvanomodsspacing.max_spacing_error;

xienomodsspacing = load("xienomodsspacing.mat", "max_spacing_error");
xienomodsspacing = xienomodsspacing.max_spacing_error;


% Define turn regions (based on the given pattern)
turn_intervals = [227 476; 627 876; 1330 1579; 1730 1979]; % Base turn intervals
cycle_length = 2206;  % The cycle repeats every 2206 iterations

% Extend the turn regions to the entire time frame
turn_times = [];
for i = 0:floor(iterations / cycle_length)  % Repeating pattern every cycle_length iterations
    turn_times = [turn_times; turn_intervals + i * cycle_length];
end
turn_times = turn_times(turn_times(:, 2) <= iterations, :); % Remove excess rows
turn_times = (turn_times-start_time)*ts;
% Create the figure
figure;

%% Plot Supremum Errors
subplot(2, 1, 1);

% Shade regions corresponding to turns
for i = 1:size(turn_times, 1)
    patch([turn_times(i, 1) turn_times(i, 2) turn_times(i, 2) turn_times(i, 1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end

% Plot data and capture handles
h1 = plot(t, silvanomodssupremum(start_time:end), 'r', 'LineWidth', 2); % Thicker lines
h2 = plot(t, silvactnomodssupremum(start_time:end), 'b', 'LineWidth', 2);
 h3 = plot(t, xienomodssupremum(start_time:end), 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 32, 'FontWeight', 'bold');
ylabel('$\mathbf{e_{\delta_0}}$ (Supremum Reference Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h1 h2 h3], {'C1 Sup. Ref Error', 'C2 Sup. Ref Error', 'C3 Sup. Ref Error'}, 'FontSize', 10); 
grid on;
xlim([1 (iterations-start_time)*ts]);
ylim([0 0.3]);
set(gca, 'FontSize', 10);

%% Plot Max Spacing Errors
subplot(2, 1, 2);

% Shade regions corresponding to turns
for i = 1:size(turn_times, 1)
    patch([turn_times(i, 1) turn_times(i, 2) turn_times(i, 2) turn_times(i, 1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end

% Plot data and capture handles
h4 = plot(t, silvanomodsspacing, 'r', 'LineWidth', 2); % Thicker lines
h5 = plot(t, silvaactnomods, 'b', 'LineWidth', 2);
h6 = plot(t, xienomodsspacing, 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 32, 'FontWeight', 'bold');
ylabel('$\mathbf{e_{\delta_i}}$ (Max Spacing Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h4 h5 h6], {'C1 Max Spacing Error', 'C2 Max Spacing Error', 'C3 Max Spacing Error'}, 'FontSize', 10);
grid on;
xlim([1 (iterations-start_time)*ts]);
ylim([0 0.15]);
set(gca, 'FontSize', 10);

% Global settings for the figure
set(gcf, 'Color', 'w');
