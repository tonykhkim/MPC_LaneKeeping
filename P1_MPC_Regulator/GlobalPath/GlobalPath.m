classdef GlobalPath
    %GLOBALPATH This class handles the global path of a vehicle.
    % It includes properties for global position coordinates and heading,
    % and methods for finding the closest path point and calculating future path points.

    properties
        GPosX; % Global Position X coordinate in meters (m)
        GPosY; % Global Position Y coordinate in meters (m)
        GHeading; % Global Heading in radians (rad)
    end
    
    methods
        function obj = GlobalPath()
            % Constructor for the GlobalPath class.
            % It loads a trajectory from a .mat file and initializes the global path properties.

            Path = load("trajectory.mat"); % Load the trajectory data
            obj.GPosX = Path.X_ref; % Initialize the X positions from the trajectory data
            obj.GPosY = Path.Y_ref; % Initialize the Y positions from the trajectory data
            obj.GHeading = Path.Psi_ref; % Initialize the headings from the trajectory data
        end
        
        function [closestPoint, closestPointIndex] = find_closest_path_point(obj, vehicle_pos_x, vehicle_pos_y, vehicleHeading)
            % This method finds the closest point on the path to a given vehicle position.
            % It takes the vehicle's position and heading as inputs and returns the closest point and its index on the path.

            path = [obj.GPosX, obj.GPosY]; % Create a matrix of path points (x, y)
            vehiclePos = [vehicle_pos_x, vehicle_pos_y]; % Vehicle's current position
        
            % Initialize the minimum distance to a large number and nearest index
            minDistance = inf;
            closestPoint = [];
            closestPointIndex = -1;
        
            % Iterate through each point on the path to find the closest point
            for i = 1:size(path, 1)
                % Compute the Euclidean distance from the vehicle to the current path point
                distance = norm(vehiclePos - path(i, :));
                
                % Update the closest point if a new minimum distance is found
                if distance < minDistance
                    minDistance = distance;
                    closestPoint = path(i, :);
                    closestPointIndex = i;
                end
            end
            
            % Return an error if no closest point is found
            if isempty(closestPoint)
                error('No nearest point found on the path.');
            end
        end

        function futurePoints = calculate_future_points(obj, nearestIndex, speed, deltaTime, Np)
            % This method calculates future points on the path based on the current speed and time delta.
            % It returns a set of points that the vehicle is expected to reach in the future.

            path = [obj.GPosX, obj.GPosY, obj.GHeading]; % Extract the path points including heading
            
            % Calculate the total distance the vehicle can travel in the given time frame
            totalDistance = speed * deltaTime;
            
            % Initialize the array to store the selected future points
            futurePoints = zeros(Np, 3);
            pointCount = 1;
            accumulatedDistance = 0;
            currentIndex = nearestIndex;

            % Iterate through the path points to find future points
            while pointCount <= Np && currentIndex < size(path, 1)
                % Calculate the distance between the current and next point
                nextIndex = currentIndex + 1;
                if nextIndex > size(path, 1)
                    break;
                end
                segmentDistance = norm(path(nextIndex, 1:2) - path(currentIndex, 1:2));
                
                % Check if the accumulated distance is within the total travel distance
                if (accumulatedDistance + segmentDistance) < totalDistance
                    accumulatedDistance = accumulatedDistance + segmentDistance;
                    currentIndex = nextIndex;
                else
                    % Interpolate a point on the segment if the total distance exceeds
                    remainingDistance = totalDistance - accumulatedDistance;
                    directionVector = (path(nextIndex, 1:2) - path(currentIndex, 1:2)) / segmentDistance;
                    interpolatedPoint = path(currentIndex, 1:2) + remainingDistance * directionVector;
                    
                    futurePoints(pointCount, 1:2) = interpolatedPoint;
                    futurePoints(pointCount, 3) = path(currentIndex, 3); % Heading of the current point
                    pointCount = pointCount + 1;
                    totalDistance = totalDistance + speed * deltaTime; % Update the total distance for the next point
                end
            end
            
            % Fill the remaining points with the last calculated point if necessary
            for i = pointCount:Np
                futurePoints(i, :) = futurePoints(pointCount - 1, :);
            end
        end
    end
end
