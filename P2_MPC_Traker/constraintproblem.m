function [A,b] = constraintproblem(mpcparam,initialinput)

    Np = mpcparam.Np; % Prediction horizon (assume that Nc=Np)
    del_u_bound = mpcparam.del_u_b; %bound for delta u constraint problem
    u_bound = mpcparam.u_b;   %bound for u constraint problem


    A_u_ub = tril(ones(Np), -1) + eye(Np);
    A_u_lb = -tril(ones(Np), -1) + eye(Np);

    b_u_ub = u_bound*ones(Np,1) - initialinput*ones(Np,1);
    b_u_lb = u_bound*ones(Np,1) + initialinput*ones(Np,1);

    % A for constraint problem of quadprog(H,F,A,b)
    A = [eye(Np); -eye(Np); A_u_ub; A_u_lb];

    % b for constraint problem of quadprog(H,F,A,b)
    b = [del_u_bound*ones(Np,1); del_u_bound*ones(Np,1); b_u_ub; b_u_lb];

    
end