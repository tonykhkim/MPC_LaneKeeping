function params=fMpcParameter()
    % Note: The parameters below can be freely modified 
    % for the design and tuning of the MPC (Model Predictive Control).

    params.Np = 20; % Number of Prediction Horizon    %Np=5
    params.Nc = params.Np; % Number of Contorl Horizon
    params.Q  = diag([5;1;5;1]);    % MPC Weight Q       %Q=diag([1;1;1;1])
    params.S  = diag([100;1;5;1]);   % Terminal Cost 
    params.R  = 1; % MPC Weight R
end