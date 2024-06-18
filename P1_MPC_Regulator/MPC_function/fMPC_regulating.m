function [u,cost] = fMPC_regulating(x, vehicleSpeed)
% FMPC_REGULATING - Function for Model Predictive Control in vehicle regulation.
%
% This function implements a Model Predictive Control strategy to regulate
% a vehicle's movement based on its current state and velocity. The function
% allows the inclusion of a terminal cost in the optimization problem.
%
% Inputs:
%   x - The current state of the vehicle.
%   vx - The current velocity of the vehicle.
% x = mpc_state(:,1);
vx = vehicleSpeed;

%
% Outputs:
%   u - The optimal control input for the vehicle.
%
% The function first loads MPC parameters and the vehicle control model. 
% Then, depending on the terminal_cost_flag, it reformulates the MPC problem 
% with or without the terminal cost. Finally, it solves the optimization 
% problem using quadratic programming and returns the first control input.

%% Req-1. Load MPC parameters (given code)
% param = fMpcParameter();
mpcparam = fMpcParameter();
% Q = param.Q; % State weighting matrix
% R = param.R; % Control weighting matrix
% Np = param.Np; % Prediction horizon (assume that Nc=Np)

%% Req-2. Obtain the discrete control model matrix
% This section retrieves the matrices (Ad, Bd, and Cd) for the discrete control model,
% representing the dynamics of the vehicle at the current velocity (vx).
% The model employed here is based on the lateral vehicle dynamic model 
% with small steering angle approximation, which is particularly effective 
% for scenarios involving minor steering adjustments.

% To accurately represent the dynamics, vehicle parameters defined in the 
% fVehParameter() function are utilized. These parameters include, but are not limited 
% to, aspects like vehicle mass, moment of inertia, wheelbase length, etc., 
% which are crucial for a realistic and effective control model.

% To Do: 
% - Implement the fVehCtrlModel function using the lateral vehicle dynamic model 
%   with the small steering angle approximation.
% - Ensure that the fVehCtrlModel function extracts necessary vehicle parameters 
%   by calling the fVehParameter() function.
% - The function should output the matrices Ad, Bd, and Cd, which are essential 
%   for the control model at the current vehicle velocity (vx).

% Example usage:
vehicleParams = fVehParameter();
[Ad, Bd, Cd] = fVehCtrlModel(vx, vehicleParams);


%% Req-3. Reformulate the given optimization problem for MPC
% This section is focused on reformulating the given optimization problem into a 
% Quadratic Programming (QP) format for the purpose of executing Model Predictive Control (MPC). 
% The function 'fReformulation_regulating' is used to transform the system's dynamics 
% (represented by matrices Ad, Bd, Cd) along with the cost matrices (Q, R) into the 
% QP format. A critical requirement for the 'fReformulation_regulating' function is its ability 
% to operate consistently across arbitrary state and output dimensions, as well as 
% for any number of prediction horizons, under the assumption that Nc equals Np.

% Notably, the Q and R matrices defined in 'fMpcParameter()' are placeholder 
% or 'dummy' values and should be adjusted to reflect the specific requirements of 
% the control problem at hand.

% The outcome of this process, 'bar_H' and 'bar_F', are the reformulated cost matrices 
% that are essential for solving the QP problem in the context of MPC.

% To Do:
% - Ensure that the 'fReformulation_regulating' function is flexible and robust enough to handle 
%   arbitrary state & output dimensions, and number of prediction horizons (assuming Nc = Np).
% - Understand and adjust the dummy values of Q and R in 'fMpcParameter()' to suit 
%   the specific needs of the MPC application.
% - Employ 'fReformulation_regulating' to convert the system dynamics and cost matrices 
%   into an appropriate format for QP.

% Example usage:
% fMpcParameter  = fVehParameter(); % Retrieving MPC parameters, including dummy Q, R
[bar_H, bar_F,A_bar,Q_bar,Q] = fReformulation_regulating(Ad,Bd,mpcparam);



%% Req-4. Solve the unconstrained MPC(Regulating) problem using quadprog
% This section uses the 'quadprog' function to solve the unconstrained MPC optimization problem.
% The objective is to minimize a quadratic cost function, 
% representing the MPC problem's cost matrices. The solution contains the sequence of 
% optimal control inputs. It is important to always check if an optimal solution exists 
% when using 'quadprog' to ensure the validity of the results.

% To Do: 
% - Use the 'quadprog' function to find the optimal solution.
% - Always check for the existence of an optimal solution when using 'quadprog'.

% Example usage:
u_optimal = quadprog(bar_H,x.'*bar_F.')   %아래의 quadprog 활용법과 결과 동일

%cost function
cost = 0.5*u_optimal.'*bar_H*u_optimal + x.'*bar_F.'*u_optimal + 0.5*x.'*(A_bar.'*Q_bar*A_bar+Q)*x;

u = u_optimal(1,1)


%% Req-6. Apply the Control Law
% This step involves applying the Control Law to the sequence of control inputs
% obtained from the optimization problem. The first element of this sequence 
% is used as the optimal control input (u) for the current state.


% u = fgenSineWave();
end
