function params=fVehParameter()
    
    % Constants    
    params.g=9.81;
    params.m=1500;
    
    params.rho=1.225;
    params.Af=1.6+0.00056*(params.m-765);
    params.Cd=0.3;
    
    params.cd=0.5*params.rho*params.Cd*params.Af;
    params.cr=0.03;
    params.Iz=3000;
    params.Caf=19000;
    params.Car=33000;
    params.lf=1.2;
    params.lr=1.6;
    params.Ts=0.01; % Cotroller Cyctle Time
    
    % Parameters for the lane change:
    % Higher psi reduces the overshoot
    % Matrix weights for the cost function (They must be diagonal)
    params.Q=[10 0;0 1]; % weights for outputs (output x output)
    params.S=[10 0;0 1]; % weights for the final horizon outputs (output x output)
    params.R=30; % weights for inputs (input x input)
    
    params.controlled_states=2;
    params.hz = 15; % horizon period
    params.x_dot=20;
    
    % Choose your trajectory (1,2)
    params.trajectory=1;
    
    % keySet={'g','m','cd','cr','Iz','Caf','Car','lf','lr','Ts','Q','S','R','controlled_states','hz','x_dot','trajectory'};
    % constants_list={g m cd cr Iz Caf Car lf lr Ts Q S R controlled_states hz x_dot trajectory};
    % params=containers.Map(keySet,constants_list);

end
