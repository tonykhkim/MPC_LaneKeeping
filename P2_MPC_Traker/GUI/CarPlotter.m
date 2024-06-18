classdef CarPlotter  < handle
    properties
        Figure
        Axis
        CarTransform % Changed: Now using hgtransform object.
        CarRectangle
        CarArrow
        PlannedPathLine
        ClosestPoint
        LegendVehicle
        LegendVelocity
        LegendPlannedPath
        Position = [0, 0]
        Heading = 0
        Velocity = 0
    end
    
    methods
        function obj = CarPlotter()
            % Fetch the full screen size
            screenSize = get(0, 'ScreenSize'); 
            % 60% of the screen width
            figWidth = screenSize(3) * 0.6; 
            % 60% of the screen height
            figHeight = screenSize(4) * 0.6;
            % X position for center alignment
            figX = (screenSize(3) - figWidth) / 2; 
            % Y position for center alignment
            figY = (screenSize(4) - figHeight) / 2; 
        
            % Set position and size
            obj.Figure = figure('Units', 'pixels', ...
                                'Position', [figX figY figWidth figHeight]); 
            obj.Axis = axes('Parent', obj.Figure, 'DataAspectRatio', [1 1 1]);
            xlim(obj.Axis, [-25 175]);
            ylim(obj.Axis, [-25 25]);
            
            % Turn on the grid
            grid(obj.Axis, 'on'); 
            % Label for X-axis
            xlabel(obj.Axis, 'X (m)'); 
            % Label for Y-axis
            ylabel(obj.Axis, 'Y (m)'); 
            % Title of the plot
            title(obj.Axis, 'Vehicle Movement Simulation'); 
        
            % Create global path line object
            obj.PlannedPathLine = line(obj.Axis, 'XData', [0,0], 'YData', [0,0], 'Color', 'b', 'LineStyle', '--');

            % Create closest point marker
            obj.ClosestPoint = line(obj.Axis, 'XData', [0,0], 'YData', [0,0], 'Color', 'r', 'Marker', '*');

            % Changed: Now using hgtransform object
            obj.CarTransform = hgtransform('Parent', obj.Axis);
            
            % Create rectangle to represent the vehicle
            obj.CarRectangle = rectangle('Position', [-2.5, -1, 5, 2], 'Parent', obj.CarTransform);
            
            % Create an arrow to indicate the heading of the vehicle
            obj.CarArrow = line('XData', [0 0], 'YData', [0 0], 'Color', 'r', 'Parent', obj.CarTransform);
            
            % Create dummy line objects for the legend
            obj.LegendVehicle = line('XData', NaN, 'YData', NaN, 'Color', 'b', 'LineWidth', 6);
            obj.LegendVelocity = line('XData', NaN, 'YData', NaN, 'Color', 'r', 'LineWidth', 2);
            % Add a dummy line for the planned path legend
            obj.LegendPlannedPath = line('XData', NaN, 'YData', NaN, 'Color', 'b', 'LineStyle', '--', 'LineWidth', 2);
        
            % Add legend
            legend([obj.LegendVehicle, obj.LegendVelocity, obj.LegendPlannedPath], {'Vehicle', 'Velocity Vector', 'Planned Path'});


            obj.updateArrow();
        end
        
        function updatePosition(obj, newPos, newHeading, newVelocity)
            % Update vehicle's position, direction, and speed
            obj.Position = newPos';
            obj.Heading = newHeading;
            obj.Velocity = newVelocity;
            
            % Move and rotate hggroup object
            set(obj.CarTransform, 'Matrix', makehgtform('translate', [newPos', 0], 'zrotate', newHeading));
            
            % Adjust the length of the arrow based on speed
            obj.updateArrow();
            
            % Move the axes if the vehicle moves out of range
            obj.updateAxis();
        end
        
        function updateArrow(obj)
            % Update the length of the arrow
            set(obj.CarArrow, 'XData', [0, obj.Velocity * 3], ...
                              'YData', [0, 0]);
        end
        
        function updateAxis(obj)
            % Axis range
            axisRangeY = 50; % 50m x 50m
            axisRangeX = 200; % 50m x 50m
            halfAxisRangeY = axisRangeY / 2;
            halfAxisRangeX = axisRangeX / 2;
            
            % Current center of the axis
            currentAxisCenter = [mean(xlim(obj.Axis)), mean(ylim(obj.Axis))];
            
            % Check if the vehicle's position is beyond the center of the current axis
            if abs(obj.Position(1) - currentAxisCenter(1)) > halfAxisRangeX
                % Update X axis
                deltaX = round((obj.Position(1) - currentAxisCenter(1)) / axisRangeX) * axisRangeX;
                newXLim = xlim(obj.Axis) + deltaX;
                xlim(obj.Axis, newXLim);
            end
            
            if abs(obj.Position(2) - currentAxisCenter(2)) > halfAxisRangeY
                % Update Y axis
                deltaY = round((obj.Position(2) - currentAxisCenter(2)) / axisRangeY) * axisRangeY;
                newYLim = ylim(obj.Axis) + deltaY;
                ylim(obj.Axis, newYLim);
            end
        end

        function updatePlannedPath(obj, gPathX, gPathY)
            % Create and store a line for the planned path
            set(obj.PlannedPathLine, 'XData', gPathX, 'YData', gPathY);
        end
    
        function updateClosestPoint(obj, posX, posY)
            % Update position of the closest point marker
            set(obj.ClosestPoint, 'XData', posX, 'YData', posY);
        end

        function delete(obj)
            % This method is called when an instance of CarPlotter is deleted.
            % It closes the figure associated with the instance.

            % Check if the figure exists and is a valid handle, then close it
            if isvalid(obj.Figure)
                close(obj.Figure);
            end
        end
    end
end

function T = makehgtform(varargin)
% This function creates a transformation matrix for position and rotation handling.
% It is a simplified version of MATLAB's built-in makehgtform function.
    T = eye(4);
    if nargin == 0
        return;
    end
    for i = 1:2:nargin
        switch varargin{i}
            case 'translate'
                T(1:3,4) = varargin{i+1};
            case 'zrotate'
                theta = varargin{i+1};
                Rz = [cos(theta), -sin(theta), 0, 0; ...
                      sin(theta), cos(theta), 0, 0; ...
                      0, 0, 1, 0; ...
                      0, 0, 0, 1];
                T = T * Rz;
        end
    end
end
