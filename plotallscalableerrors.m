close all;
clear all;

% SilvaActLostComms
% SilvaLostComms
% XieLostComms

close all;
clear all;
platoon_sizes = [4,8,12,16,20];

% Initialize matrices to store errors for all platoon sizes
supremum_errors = zeros(length(platoon_sizes), 3);  % 3 methods
spacing_errors = zeros(length(platoon_sizes), 3);   % 3 methods

% After the for-loop ends
figure;

%% Plot Supremum Reference Error (Line Plot) %%
subplot(2,1,1);
hold on;

% Plot for SilvaNomods (Implementation 2)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_silva", N));  % Load each saved error

    % Store for bar plot
    supremum_errors(idx, 1) = maxsupremreferrortotalsilvalostcomms;
    
    % Plot for SilvaNomods
    h2 = plot(N, maxsupremreferrortotalsilvalostcomms, 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);  % Green dots
    line([N N], [0 maxsupremreferrortotalsilvalostcomms], 'LineStyle', '--', 'Color', 'g');
end

% Plot for SilvaActNomods (Implementation 3)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_silvact", N));  % Load each saved error

    % Store for bar plot
    supremum_errors(idx, 2) = maxsupremreferrortotalsilvactlostcomms;
    
    % Plot for SilvaActNomods
    h3 = plot(N, maxsupremreferrortotalsilvactlostcomms, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 8);  % Red dots
    line([N N], [0 maxsupremreferrortotalsilvactlostcomms], 'LineStyle', '--', 'Color', 'r');
end

% Plot for XieNomods (Implementation 1)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_xie", N));  % Load each saved error

    % Store for bar plot
    supremum_errors(idx, 3) = maxsupremreferrortotalxielostcomms;
    
    % Plot for XieNomods
    h1 = plot(N, maxsupremreferrortotalxielostcomms, 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8);  % Blue dots
    line([N N], [0 maxsupremreferrortotalxielostcomms], 'LineStyle', '--', 'Color', 'b');
end

% Update title, axis labels, and legend
title('Supremum Reference Error for Different Platoon Sizes', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Platoon Size (N)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Supremum Error [m]', 'FontSize', 10, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);  % Update tick size to IEEE standard
lgd = legend([h2 h3 h1], {'Method 1', 'Method 2', 'Method 3'}, 'FontSize', 10, 'Location', 'best');
grid on;
hold off;

%% Plot Max Spacing Error (Line Plot) %%
subplot(2,1,2);
hold on;

% Plot for SilvaNomods (Implementation 2)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_silva", N));  % Load each saved error

    % Store for bar plot
    spacing_errors(idx, 1) = maxspacingerrortotalsilvalostcomms;
    
    % Plot for SilvaNomods
    h2 = plot(N, maxspacingerrortotalsilvalostcomms, 'ko', 'MarkerFaceColor', 'g', 'MarkerSize', 8);  % Green dots
    line([N N], [0 maxspacingerrortotalsilvalostcomms], 'LineStyle', '--', 'Color', 'g');
end

% Plot for SilvaActNomods (Implementation 3)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_silvact", N));  % Load each saved error

    % Store for bar plot
    spacing_errors(idx, 2) = maxspacingerrortotalsilvactlostcomms;
    
    % Plot for SilvaActNomods
    h3 = plot(N, maxspacingerrortotalsilvactlostcomms, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 8);  % Red dots
    line([N N], [0 maxspacingerrortotalsilvactlostcomms], 'LineStyle', '--', 'Color', 'r');
end

% Plot for XieNomods (Implementation 1)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_xie", N));  % Load each saved error

    % Store for bar plot
    spacing_errors(idx, 3) = maxspacingerrortotalxielostcomms;
    
    % Plot for XieNomods
    h1 = plot(N, maxspacingerrortotalxielostcomms, 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8);  % Blue dots
    line([N N], [0 maxspacingerrortotalxielostcomms], 'LineStyle', '--', 'Color', 'b');
end

% Update title, axis labels, and legend
title('Max Spacing Error for Different Platoon Sizes', 'FontSize', 10, 'FontWeight', 'bold');
xlabel('Platoon Size (N)', 'FontSize', 10, 'FontWeight', 'bold');
ylabel('Max Spacing Error [m]', 'FontSize', 10, 'FontWeight', 'bold');
set(gca, 'FontSize', 10);  % Update tick size to IEEE standard
lgd = legend([h2 h3 h1], {'Method 1', 'Method 2', 'Method 3'}, 'FontSize', 10, 'Location', 'best');
grid on;
hold off;

%% Bar Graph for Supremum Reference Error and Max Spacing Error %%
% Create the figure for Supremum Reference Error and Max Spacing Error
figure;

% Plot Supremum Reference Error as bar plot
subplot(2,1,1);
bar(platoon_sizes, supremum_errors);

% X-axis label
xlb = xlabel('Platoon Size (N)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

% Y-axis label with LaTeX interpreter
ylb = ylabel('$\mathbf{e_{\delta_0}}$ (Supremum Reference Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

% Legend with increased font size
lgd = legend({'C1','C2','C3'}, 'FontSize', 10, 'Location', 'best');
grid on;

% Set font size and weight for axis ticks
set(gca, 'FontSize', 10, 'FontWeight', 'bold');

% Plot Max Spacing Error as bar plot
subplot(2,1,2);
bar(platoon_sizes, spacing_errors);

% X-axis label
xlb = xlabel('Platoon Size (N)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

% Y-axis label with LaTeX interpreter
ylb = ylabel('$\mathbf{e_{\delta_i}}$ (Max Spacing Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

% Legend with increased font size
lgd = legend({'C1','C2','C3'}, 'FontSize', 10, 'Location', 'best');
grid on;

% Set font size and weight for axis ticks
set(gca, 'FontSize', 10, 'FontWeight', 'bold');
