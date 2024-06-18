function params=fMpcParameter()
    % Note: The parameters below can be freely modified 
    % for the design and tuning of the MPC (Model Predictive Control).
    params.Np = 10; % Number of Prediction Horizon
    params.Nc = params.Np; % Number of Contorl Horizon
    % params.Q  = diag([8; 0.2; 10; 0.2]); % MPC Weight Q
    params.Q  = diag([100; 1; 50; 1]); % MPC Weight Q    %Q=diag([1; 1; 1; 1]);
    params.R  = 1; % MPC Weight R
    params.Ts=0.01; % Cotroller Cyctle Time
    params.del_u_b = 0.019; %bound of delta u_max for constraint problem  %arbitrary = 0.01  %desired = 0.019
    params.u_b = 0.5; %bound of u for constraint problem
end