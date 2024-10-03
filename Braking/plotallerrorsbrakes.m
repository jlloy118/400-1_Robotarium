close all;
clear all;

% Load the necessary data
% XieRussoBrakes;
% SilvaBrakes;
% Method2Brakes;

close all;
clear all;
ts = 0.033;
iterations = 4000;
start_time = 4000-3001+1;
t = 1:iterations-start_time+1;
t = t *ts;

silvanomodsupremum = load("supremerrorsilvabrakes.mat", "supremum_reference_error");
silvanomodssupremum = silvanomodsupremum.supremum_reference_error;

xienomodssupremum = load("supremerrorxiebrakes.mat", "supremum_reference_error");
xienomodssupremum = xienomodssupremum.supremum_reference_error;

silvactnomodssupremum = load("silvactbrakessupremum.mat", "supremum_reference_error");
silvactnomodssupremum = silvactnomodssupremum.supremum_reference_error;

silvaactnomods = load("silvactbrakesspacing.mat", "max_spacing_error");
silvaactnomods = silvaactnomods.max_spacing_error;

silvanomodsspacing = load("silvabrakesspacing.mat", "max_spacing_error");
silvanomodsspacing = silvanomodsspacing.max_spacing_error;

xienomodsspacing = load("xiebrakessspacing.mat", "max_spacing_error");
xienomodsspacing = xienomodsspacing.max_spacing_error;


% Define braking time interval
braking_start = 930;
braking_end = braking_start + 243;
braking_cycle_length = 2595;



% Extend the braking regions to the entire time frame
braking_times = [];
for i = 0:floor(iterations / braking_cycle_length)
    braking_times = [braking_times; braking_start + i * braking_cycle_length, braking_end + i * braking_cycle_length];
end
braking_times = braking_times(braking_times(:, 2) <= iterations, :);

% Define turn regions
turn_intervals = [227 476; 627 876; 1330 1579; 1730 1979];
turn_cycle_length = 2206;

% Extend the turn regions to the entire time frame
turn_times = [];
for i = 0:floor(iterations / turn_cycle_length)
    turn_times = [turn_times; turn_intervals + i * turn_cycle_length];
end
turn_times = turn_times(turn_times(:, 2) <= iterations, :);
turn_times = (turn_times-start_time)*ts;
braking_times = (braking_times -start_time)*ts;
% Create the figure
figure;

%% Plot Supremum Errors
subplot(2, 1, 1);

% Shade turn regions
for i = 1:size(turn_times, 1)
    patch([turn_times(i, 1) turn_times(i, 2) turn_times(i, 2) turn_times(i, 1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end

for i = 1:size(braking_times, 1)
    patch([braking_times(i, 1) braking_times(i, 2) braking_times(i, 2) braking_times(i, 1)], ...
          [0 0 1 1], [1 1 0.6], 'EdgeColor', 'none');
    hold on;
end



% Plot data
h1 = plot(t, silvanomodssupremum(start_time:end), 'r', 'LineWidth', 2);
h2 = plot(t, silvactnomodssupremum(start_time:end), 'b', 'LineWidth', 2);
h3 = plot(t, xienomodssupremum(start_time:end), 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('$\mathbf{e_{\delta_0}}$ (Supremum Reference Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h1 h2 h3], {'C1 Sup. Ref Error', 'C2 Sup. Ref Error', 'C3 Sup. Ref Error'}, 'FontSize', 10);
grid on;
xlim([0 max(t)])
ylim([0 0.35]);
set(gca, 'FontSize', 10);

%% Plot Max Spacing Errors
subplot(2, 1, 2);


% Shade turn regions
for i = 1:size(turn_times, 1)
    patch([turn_times(i, 1) turn_times(i, 2) turn_times(i, 2) turn_times(i, 1)], ...
          [0 0 1 1], [0.9 0.9 0.9], 'EdgeColor', 'none');
    hold on;
end

% Shade braking regions
for i = 1:size(braking_times, 1)
    patch([braking_times(i, 1) braking_times(i, 2) braking_times(i, 2) braking_times(i, 1)], ...
          [0 0 1 1], [1 1 0.6], 'EdgeColor', 'none');
    hold on;
end



% Plot data
h4 = plot(t, silvanomodsspacing, 'r', 'LineWidth', 2);
h5 = plot(t, silvaactnomods, 'b', 'LineWidth', 2);
h6 = plot(t, xienomodsspacing, 'g', 'LineWidth', 2);

% Formatting
xlabel('t (s)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('$\mathbf{e_{\delta_i}}$ (Max Spacing Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');
legend([h4 h5 h6], {'C1 Max Spacing Error', 'C2 Max Spacing Error', 'C3 Max Spacing Error'}, 'FontSize', 10);
grid on;
ylim([0 0.35]);
xlim([0 max(t)])
set(gca, 'FontSize', 10);

% Global settings for the figure
set(gcf, 'Color', 'w');