close all;
clear all;
% SilvaActLostComms;
% SilvaLostComms;
% XieLostComms;
% Load the necessary data
clear all
close all
start_time = 850;
iterations = 4000;
ts = 0.033;
t = 1:iterations-start_time + 1;
t = t.*ts;


silvanomodsupremum = load("supremerrorsilvablackout.mat","supremum_reference_error");
silvanomodssupremum = silvanomodsupremum.supremum_reference_error;

xienomodssupremum = load("supremerrorxieblackout.mat","supremum_reference_error");
xienomodssupremum = xienomodssupremum.supremum_reference_error;

silvactnomodssupremum = load("supremumreferrorsilvaactblackout.mat","supremum_reference_error");
silvactnomodssupremum = silvactnomodssupremum.supremum_reference_error;

silvaactnomods = load("silvaactsupremumspacingblackout.mat","max_spacing_error");
silvaactnomods = silvaactnomods.max_spacing_error;

silvanomodsspacing = load("silvablackoutspacing.mat","max_spacing_error");
silvanomodsspacing = silvanomodsspacing.max_spacing_error;

xienomodsspacing = load("spacingerrorxieblackout.mat","max_spacing_error");
xienomodsspacing = xienomodsspacing.max_spacing_error;

blackout_start = 3000;
blackout_end = 3000 + 3/ts;

% Define turn regions (based on the given pattern)
turn_intervals = [227 476; 627 876; 1330 1579; 1730 1979]; % Base turn intervals
cycle_length = 2206;  % The cycle repeats every 2206 iterations

% Extend the turn regions to the entire time frame
turn_times = [];
for i = 0:floor(iterations / cycle_length)  % Repeating pattern every cycle_length iterations
    turn_times = [turn_times; turn_intervals + i * cycle_length];
end
turn_times = turn_times(turn_times(:,2) <= iterations, :); % Remove excess rows
turn_times = turn_times*ts;
% Create the figure
figure;

blackout_start = blackout_start - start_time;
blackout_end = blackout_end - start_time;

%% Plot Supremum Errors
subplot(2, 1, 1);

% Shade regions corresponding to turns in grey
for i = 1:size(turn_times,1)
    patch([turn_times(i,1) turn_times(i,2) turn_times(i,2) turn_times(i,1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end
% Shade blackout region in light yellow
patch([blackout_start*ts blackout_end*ts blackout_end*ts blackout_start*ts], ...
      [0 0 1 1], [1 1 0.6], 'EdgeColor', 'none');
  
% Plot data and capture handles
h1 = plot(t, silvanomodssupremum(start_time:end), 'r', 'LineWidth',2); % Thicker lines
h2 = plot(t, silvactnomodssupremum(start_time:end), 'b', 'LineWidth',2);
h3 = plot(t, xienomodssupremum(start_time:end), 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylb = ylabel('$\mathbf{e_{\delta_0}}$ (Supremum Reference Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h1 h2 h3], {'C1 Sup. Ref Error', 'C2 Sup. Ref Error', 'C3 Sup. Ref Error'}, 'FontSize', 10); 
grid on;
xlim([1 (iterations-start_time)*ts]);
ylim([0 0.3]);
set(gca, 'FontSize', 10);  % Bigger font for axes ticks

%% Plot Max Spacing Errors
subplot(2, 1, 2);

% Shade regions corresponding to turns in grey
for i = 1:size(turn_times,1)
    patch([turn_times(i,1) turn_times(i,2) turn_times(i,2) turn_times(i,1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end
% Shade blackout region in light yellow
patch([blackout_start*ts blackout_end*ts blackout_end*ts blackout_start*ts], ...
      [0 0 1 1], [1 1 0.6], 'EdgeColor', 'none');

% Plot data and capture handles
h4 = plot(t, silvanomodsspacing, 'r', 'LineWidth', 2); % Thicker lines
h5 = plot(t, silvaactnomods, 'b', 'LineWidth', 2);
h6 = plot(t, xienomodsspacing, 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 32, 'FontWeight', 'bold');
ylb = ylabel('$\mathbf{e_{\delta_i}}$ (Max Spacing Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h4 h5 h6], {'C1 Max Spacing Error', 'C2 Max Spacing Error', 'C3 Max Spacing Error'}, 'FontSize', 10);
grid on;
xlim([1 (iterations-start_time)*ts]);
ylim([0 0.3]);
set(gca, 'FontSize', 10);  % Bigger font for axes ticks

% Global settings for the figure
set(gcf, 'Color', 'w');  % Set background color of the figure to white for better printing
