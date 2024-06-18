function [hAxis1, hAxis2, hAxis3, hAxis4] = initializeStatePlot()
    % Initialize the GUI for plotting
    close all;
    hFig = figure("Name", "State Error");
    hAxis1 = subplot(4, 1, 1, "Parent", hFig); grid(hAxis1, "on");
    hAxis2 = subplot(4, 1, 2, 'Parent', hFig); grid(hAxis2, "on");
    hAxis3 = subplot(4, 1, 3, 'Parent', hFig); grid(hAxis3, "on");
    hAxis4 = subplot(4, 1, 4, 'Parent', hFig); grid(hAxis4, "on");
end

