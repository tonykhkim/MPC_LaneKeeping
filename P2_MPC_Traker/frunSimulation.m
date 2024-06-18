function state_array = frunSimulation()
    % Initialize simulation parameters
    params = fSimParameter();
    startTime = params.SimStartTime; % Start time of the simulation
    endTime = params.SimEndTime; % End time of the simulation
    timeStep = params.SimStepTime; % Time step for the simulation
    timeArray = startTime:timeStep:endTime; % Array of simulation time steps
    vehicleSpeed = params.VehicleSpeedMps; % Vehicle speed in meters per second
    visualizationFreq = 10; % Frequency for visualization updates in Hz
    visualizationInterval = 1 / visualizationFreq; % Time interval for visualization updates in seconds

    % Initialize Controller parameters
    params = fMpcParameter();
    Ts = params.Ts; % Cotroller Cyctle Time
    Np = params.Np; % Number of Prediction Horizon

    % Instantiate GlobalPath, PlantModel, and CarPlotter objects
    globalPath = GlobalPath();
    plantModel = PlantModel();
    carPlotter = CarPlotter();

    % Update the global path in the GUI
    carPlotter.updatePlannedPath(globalPath.GPosX, globalPath.GPosY);

    % Initialize the vehicle state [PosX, PosY, Vy, Yaw, Yawrate]
    State = [0; -3; 0; 0; 0];
    steeringAngle = 0;
    state_array = zeros(length(timeArray), length(State));

    % plot for cost function
    cost_array = zeros(length(timeArray),1);

    ctrl_ref = [];
    % Simulation loop
    for idx = 1:length(timeArray)
        currentTime = timeArray(idx);
        % Find the closest point on the path to the vehicle
        [closestPoint, closestIndex]= globalPath.find_closest_path_point(State(1), State(2), State(4));
        referencePoints = globalPath.calculate_future_points(closestIndex, vehicleSpeed, Ts, Np)';
        % Calculate the steering angle using Model Predictive Control (MPC)
        mpc_state = [State(2:end);steeringAngle]; % state for lateral dynamic model [PosY, Vy, Yaw, Yawrate]
        ctrl_ref(1,:) = referencePoints(2,:);
        ctrl_ref(2,:) = zeros(1,length(referencePoints(2,:)));
        ctrl_ref(3,:) = referencePoints(3,:);
        ctrl_ref(4,:) = zeros(1,length(referencePoints(2,:)));
        [steeringAngle,cost] = fMPC_tracking(mpc_state, vehicleSpeed, ctrl_ref);
        cost_array(idx,1) = cost;
        % Simulate the vehicle dynamics
        dynamicsTimeSpan = 0:0.01:0.01;
        [~, stateEvolution] = plantModel.solveDynamics(dynamicsTimeSpan, State, steeringAngle, vehicleSpeed);
        State = stateEvolution(end,:)';

        % Record the state of the vehicle at each time step
        state_array(idx, :) = State;

        % error_array(idx, :) = error.';
        
        % Update the closest path point in the GUI
        carPlotter.updateClosestPoint(closestPoint(1), closestPoint(2));
        
        % Update the vehicle's position, orientation, and speed in the GUI
        carPlotter.updatePosition(State(1:2), State(4), vehicleSpeed);
        
        % Limit the frequency of drawnow calls to optimize performance
        if mod(currentTime, visualizationInterval) < timeStep
%             drawnow; % Update the GUI periodically based on visualization interval
            pause(0.0001)
        end
    end
    figure('Name','cost function')
    plot(timeArray,cost_array)
    grid on

    pause(1)
end
