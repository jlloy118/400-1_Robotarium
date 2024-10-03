% close all;
% clear all;
SilvaBrakes;
Method2Brakes;
XieRussoBrakes;

close all;
clear all;
platoon_sizes = [4,8,12,16,20];
% After the for-loop ends
figure;

%% Plot Supremum Reference Error %%
subplot(2,1,1);
hold on;



% Plot for SilvaNomods (Implementation 2)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_silva", N));  % Load each saved error

    % Plot for SilvaNomods
    h2 = plot(N, maxsupremreferrortotalsilvabrakes, 'ko', 'MarkerFaceColor', 'g');  % Green dots
    % Vertical dotted line
    line([N N], [0 maxsupremreferrortotalsilvabrakes], 'LineStyle', '--', 'Color', 'g');
end

% Plot for SilvaActNomods (Implementation 3)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_silvact", N));  % Load each saved error

    % Plot for SilvaActNomods
    h3 = plot(N, maxsupremreferrortotalsilvactnomods, 'ko', 'MarkerFaceColor', 'r');  % Red dots
    % Vertical dotted line
    line([N N], [0 maxsupremreferrortotalsilvactnomods], 'LineStyle', '--', 'Color', 'r');
end

% Plot for XieNomods (Implementation 1)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("supremum_error_platoon_size_%d_xie", N));  % Load each saved error

    % Plot for XieNomods
    h1 = plot(N, maxsupremreferrortotalxienomods, 'ko', 'MarkerFaceColor', 'b');  % Blue dots
    % Vertical dotted line
    line([N N], [0 maxsupremreferrortotalxienomods], 'LineStyle', '--', 'Color', 'b');
end

xlabel('Platoon Size (N)');
ylabel('Supremum Error [m]');
lgd = legend([ h2 h3  h1], {'C1','C2','C3'});
% Adjust the legend with bigger font
lgd.FontSize = 10;  % Increase the font size
lgd.Location = 'best';  % Optional: Move the legend to the best position
hold off;


%% Plot Max Spacing Error %%
subplot(2,1,2);
hold on;



% Plot for SilvaNomods (Implementation 2)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_silva", N));  % Load each saved error

    % Plot for SilvaNomods
    h2 = plot(N, maxsupremreferrortotalsilvabrakes, 'ko', 'MarkerFaceColor', 'g','MarkerSize',8);  % Green dots
    % Vertical dotted line
    line([N N], [0 maxsupremreferrortotalsilvabrakes], 'LineStyle', '--', 'Color', 'g');
end

% Plot for SilvaActNomods (Implementation 3)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_silvact", N));  % Load each saved error

    % Plot for SilvaActNomods
    h3 = plot(N, maxspacingerrortotalsilvactnomods, 'ko', 'MarkerFaceColor', 'r','MarkerSize',8);  % Red dots
    % Vertical dotted line
    line([N N], [0 maxspacingerrortotalsilvactnomods], 'LineStyle', '--', 'Color', 'r');
end
% Plot for XieNomods (Implementation 1)
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    load(sprintf("spacing_error_platoon_size_%d_xie", N));  % Load each saved error

    % Plot for XieNomods
    h1 = plot(N, maxspacingerrortotalxienomods, 'ko', 'MarkerFaceColor', 'b','MarkerSize',8);  % Blue dots
    % Vertical dotted line
    line([N N], [0 maxspacingerrortotalxienomods], 'LineStyle', '--', 'Color', 'b');
end

xlabel('Platoon Size (N)');
ylabel('Max Spacing Error [m]');
lgd = legend([ h2 h3  h1], {'C1','C2','C3'});
% Adjust the legend with bigger font
lgd.FontSize = 10;  % Increase the font size
lgd.Location = 'best';  % Optional: Move the legend to the best position
hold off;
hold off;

% Initialize matrices to store errors for all platoon sizes
supremum_errors = zeros(length(platoon_sizes), 3);  % 3 methods
spacing_errors = zeros(length(platoon_sizes), 3);  % 3 methods

% Loop through each platoon size and load the errors for all methods
for idx = 1:length(platoon_sizes)
    N = platoon_sizes(idx);
    
    % Load supremum and spacing errors for Method 1 (Xie)
    load(sprintf("supremum_error_platoon_size_%d_xie", N));
    load(sprintf("spacing_error_platoon_size_%d_xie", N));
    supremum_errors(idx, 3) = maxsupremreferrortotalxienomods;
    spacing_errors(idx, 3) = maxspacingerrortotalxienomods;
    
    % Load supremum and spacing errors for Method 2 (SilvaNomods)
    load(sprintf("supremum_error_platoon_size_%d_silva", N));
    load(sprintf("spacing_error_platoon_size_%d_silva", N));
    supremum_errors(idx, 1) = maxsupremreferrortotalsilvabrakes;
    spacing_errors(idx, 1) = maxspacingerrortotalsilvabrakes;
    
    % Load supremum and spacing errors for Method 3 (SilvaActNomods)
    load(sprintf("supremum_error_platoon_size_%d_silvact", N));
    load(sprintf("spacing_error_platoon_size_%d_silvact", N));
    supremum_errors(idx, 2) = maxsupremreferrortotalsilvactnomods;
    spacing_errors(idx, 2) = maxspacingerrortotalsilvactnomods;
end

% Adjust the plot for readability with bold labels, bigger ticks, and larger font sizes

% Create the figure for Supremum Reference Error and Max Spacing Error
figure;

% Plot Supremum Reference Error as bar plot
subplot(2,1,1);
bar(platoon_sizes, supremum_errors);

% X-axis label
xlb = xlabel('Platoon Size (N)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

% Y-axis label with LaTeX interpreter
ylb = ylabel('$\mathbf{e_{\delta_i}}$ (Supremum Error)', 'Interpreter', 'latex', 'FontSize', 10, 'FontWeight', 'bold');

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