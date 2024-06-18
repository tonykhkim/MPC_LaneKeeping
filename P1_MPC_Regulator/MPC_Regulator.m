clear all;
clc; 

% Initialize Script
clear all
close all
[folderPath,~,~]=fileparts(mfilename("fullpath"));
addpath(genpath(folderPath));

% Initialize State Plot
[hAxis1, hAxis2, hAxis3, hAxis4] = initializeStatePlot();


% Run simulation
state_array = frunSimulation();

% Plot each state
fplotState(hAxis1, state_array, 2); % Y Error
fplotState(hAxis2, state_array, 3); % vy Error
fplotState(hAxis3, state_array, 4); % Heading Error
fplotState(hAxis4, state_array, 5); % Yawrate Error

% call clean up function
cleanupPath(folderPath)

function cleanupPath(folderPath)
    % Remove the specified path and all its subfolders from the MATLAB path
    rmpath(genpath(folderPath));
end
