clear all;


% Specify the directory and filename
directory = 'YOUR PATH';
% Fill your path above
fullFilePath = fullfile(directory, 'measurements.csv'); % Constructs full file path


% Read data from CSV. Assuming there are no headers, we use 'VariableNamingRule','preserve'
opts = detectImportOptions(fullFilePath, 'VariableNamingRule', 'preserve');
data = readtable(fullFilePath, opts);

% Extract raw and averaged distances directly from the table
rawDistance = data{:, 1}; % First column for raw distance
measuredDistance = data{:, 2}; % Second column for average distance
R = data{:, 3}; % Second column for average distance


% Specify the number of samples to collect
numSamples = height(data); 
samplingInterval = 60e-3; % 60 milliseconds per sample
% Calculate the time for each sample based on its index
times = (0:numSamples-1) * samplingInterval;

distance = data{:,1};
newDistance = distance;
% Iterate through the newDistance array starting from the second element
for i = 2:length(newDistance)
    if newDistance(i) > 400
        newDistance(i) = newDistance(i-1); % Replace with the previous value
    end
end

subplot(1,2,1)
hold on; % Allows multiple plots on the same figure
fig1_comps.p1 = plot(times, rawDistance, 'o-', 'DisplayName', 'Measured Distances');
fig1_comps.p2 = plot(times, measuredDistance, 's-', 'DisplayName', 'Filtered Distances');
hold off;

% Labeling the plot
xlabel('Time (seconds)');
ylabel('Distance (cm)');
title('Distance Measurement Comparison');
legend('show'); % Display legend
set(gca,'fontsize',18);
set(gca,'linewidth',4);
set(gcf,'color','w');
ax = gca;
ax.FontSize = 20;  % Font Size of 20
grid on;




subplot(1,2,2);
hold on;
fig1_comps.p3 = plot(times,R, 's-', 'DisplayName', 'Filtered Distances');
title('R Coefficent');
set(gca,'fontsize',18);
set(gca,'linewidth',4);
set(gcf,'color','w');
ax = gca;
ax.FontSize = 20;  % Font Size of 20
hold off;
grid on;

% Set aesthetics for plots
set(fig1_comps.p1, 'LineStyle', '-', 'Color', PS.Black, 'LineWidth', 2);
set(fig1_comps.p2, 'LineStyle', '-', 'Color', PS.Green2, 'LineWidth', 1);
set(fig1_comps.p3, 'LineStyle', '-', 'Color', PS.Red2, 'LineWidth', 1);

% INSTANTLY IMPROVE AESTHETICS-most important step
STANDARDIZE_FIGURE(fig1_comps)

