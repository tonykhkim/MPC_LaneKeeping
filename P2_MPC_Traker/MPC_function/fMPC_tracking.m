function [u,cost] = fMPC_tracking(x, vx, rG)
% FMPC_REGULATING - Function for Model Predictive Control in vehicle regulation.
%
% This function implements a Model Predictive Control strategy to regulate
% a vehicle's movement based on its current state and velocity. The function
% allows the inclusion of a terminal cost in the optimization problem.
%
% Inputs:
%   x - The current state of the vehicle.
%   vx - The current velocity of the vehicle.

%
% Outputs:
%   u - The optimal control input for the vehicle.
%
% The function first loads MPC parameters and the vehicle control model. 
% Then, depending on the terminal_cost_flag, it reformulates the MPC problem 
% with or without the terminal cost. Finally, it solves the optimization 
% problem using quadratic programming and returns the first control input.

%% Req-1. Load MPC parameters (given code)
param = fMpcParameter();
Q = param.Q; % State weighting matrix
R = param.R; % Control weighting matrix
Np = param.Np; % Prediction horizon (assume that Nc=Np)
Ts = param.Ts; % Number of Prediction Horizon


%% Req-2. Obtain the Discrete Augmented Control Model Matrices
% This section focuses on deriving the matrices (Ad, Bd, and Cd) for the discrete augmented control model,
% which characterizes the dynamics of the vehicle at its current velocity (vx).
% The augmented model extends the standard model by incorporating the input as part of the state and
% using the change in input (Delta Input) as the actual input.

% The approach is grounded in the lateral vehicle dynamic model, tailored with the small steering angle approximation.
% This approximation is highly effective in scenarios where only minor steering adjustments are made.

% For an accurate representation of the dynamics, the vehicle parameters outlined in the
% fVehParameter() function are crucial. These parameters include vehicle mass, moment of inertia,
% wheelbase length, among others, and are essential for constructing a realistic and effective augmented control model.

% To Do:
% - Adapt the fVehCtrlModel function to align with the discrete augmented control model.
% This includes incorporating the input as a part of the state and focusing on Delta Input.
% - Ensure that the adapted fVehCtrlModel function continues to call the fVehParameter() function
% to obtain necessary vehicle parameters.
% - The function should output the matrices A_aug, B_aug, and C_aug, which are now tailored to
% the discrete augmented control model at the current vehicle velocity (vx).

% Example usage:
vehicleParams = fVehParameter();
[A_aug, B_aug, C_aug] = fVehCtrlModel(vx, vehicleParams);

%% Req-3. Reformulate the given optimization problem for MPC
%% This section is dedicated to reformulating the given control problem for designing a controller that tracks a reference (rG) using a Quadratic Programming (QP) solver in the context of Model Predictive Control (MPC).
% The function 'fReformulation_tracking' is essential for this process. It transforms the system dynamics
% (represented by matrices Ad, Bd, Cd) and the cost matrices (Q, R) into the QP format,
% specifically tailored for reference tracking.

% A key aspect of the 'fReformulation_tracking' function is its capacity to handle various state and output dimensions,
% and accommodate any number of prediction horizons, with the assumption that Nc equals Np.

% It's important to note that the Q and R matrices defined in 'fMpcParameter()' are initially set as placeholder
% values and must be fine-tuned to align with the specific objectives of the reference tracking control problem.

% The output of this reformulation, 'bar_H' and 'bar_F', are the restructured cost matrices
% crucial for solving the QP problem within MPC for reference tracking.

% To Do:
% - Develop the 'fReformulation_tracking' function to be flexible and robust, capable of handling
% varying state & output dimensions, and different numbers of prediction horizons (assuming Nc = Np).
% - Modify and optimize the initial values of Q and R in 'fMpcParameter()' to effectively address
% the needs of the reference tracking application in MPC.
% - Utilize 'fReformulation_tracking' to convert the system dynamics and cost matrices
% into the appropriate QP format for reference tracking.

% Example usage:
% fMpcParameter = mpcParams; % Retrieving MPC parameters, including tailored Q, R
[bar_H, transpose_bar_F]=fReformulation_tracking(A_aug,B_aug,C_aug,param);


%% Req-4. Solve the unconstrained MPC(Regulating) problem using quadprog
% This section uses the 'quadprog' function to solve the unconstrained MPC optimization problem.
% The objective is to minimize a quadratic cost function, 
% representing the MPC problem's cost matrices. The solution contains the sequence of 
% optimal control inputs. It is important to always check if an optimal solution exists 
% when using 'quadprog' to ensure the validity of the results.

% To Do: 
% - Use the 'quadprog' function to find the optimal solution.
% - Always check for the existence of an optimal solution when using 'quadprog'.


%% Req-5. add the QP Solver Linear Constraint Problem using quadprog
reference = rG(:);    % Control reference
[A,b] = constraintproblem(param,x(5));
u_optimal2 = quadprog(bar_H,[x' reference']*transpose_bar_F,A,b);   %delta u
u = x(5) + u_optimal2(1);





%cost function
cost = 0.5*u_optimal2.'*bar_H*u_optimal2 + [x' reference']*transpose_bar_F*u_optimal2 + x.'*C_aug.'*Q*C_aug*x - rG(:,1).'*Q*C_aug*x;  %


%% Req-6. Apply the Control Law
% This step involves applying the Control Law to the sequence of control inputs
% obtained from the optimization problem. The first element of this sequence 
% is used as the optimal control input (u) for the current state.

% u = fgenSineWave();
end
