function state_array = frunSimulation()
    % Initialize simulation parameters
    params = fSimParameter();
    startTime = params.SimStartTime; % Start time of the simulation
    endTime = params.SimEndTime; % End time of the simulation
    timeStep = params.SimStepTime; % Time step for the simulation
    timeArray = startTime:timeStep:endTime; % Array of simulation time steps
    vehicleSpeed = params.VehicleSpeedMps; % Vehicle speed in meters per second
    visualizationFreq = 30; % Frequency for visualization updates in Hz
    visualizationInterval = 1 / visualizationFreq; % Time interval for visualization updates in seconds

    % Instantiate GlobalPath, PlantModel, and CarPlotter objects
    globalPath = GlobalPath();
    plantModel = PlantModel();
    carPlotter = CarPlotter();

    % Update the global path in the GUI
    carPlotter.updatePlannedPath(globalPath.GPosX, globalPath.GPosY);

    % Initialize the vehicle state [PosX, PosY, Vy, Yaw, Yawrate]
    State = [0; 1; 0; 0; 0];
    state_array = zeros(length(timeArray), length(State));

    % plot for input signal
    input_array = zeros(length(timeArray),1);

    % plot for cost function
    cost_array = zeros(length(timeArray),1);

    % Simulation loop
    for idx = 1:length(timeArray)
        currentTime = timeArray(idx);
        % Calculate the steering angle using Model Predictive Control (MPC)
        mpc_state = State(2:end); % state for lateral dynamic model [PosY, Vy, Yaw, Yawrate]
        [steeringAngle,cost] = fMPC_regulating(mpc_state, vehicleSpeed);
        input_array(idx,1) = steeringAngle;
        cost_array(idx,1) = cost;
        % Simulate the vehicle dynamics
        dynamicsTimeSpan = 0:0.01:0.01;
        [~, stateEvolution] = plantModel.solveDynamics(dynamicsTimeSpan, State, steeringAngle, vehicleSpeed);
        State = stateEvolution(end,:)';

        % Record the state of the vehicle at each time step
        state_array(idx, :) = State;
        % Find the closest point on the path to the vehicle
        closestPoint = globalPath.find_closest_path_point(State(1), State(2), State(4));
        % Update the closest path point in the GUI
        carPlotter.updateClosestPoint(closestPoint(1), closestPoint(2));
        
        % Update the vehicle's position, orientation, and speed in the GUI
        carPlotter.updatePosition(State(1:2), State(4), vehicleSpeed);
        
        % Limit the frequency of drawnow calls to optimize performance
        if mod(currentTime, visualizationInterval) < timeStep
%             drawnow; % Update the GUI periodically based on visualization interval
            pause(0.001)
        end
    end
    figure('Name','input signal')
    plot(timeArray,input_array)
    grid on

    figure('Name','cost function')
    plot(timeArray,cost_array)
    grid on
    
    pause(1)
end
