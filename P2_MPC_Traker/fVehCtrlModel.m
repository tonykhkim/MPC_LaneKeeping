function [A_aug_d, B_aug_d, C_aug_d] = fVehCtrlModel(vx, param)

    %augmented model
    A_aug = zeros(5,5);
    B_aug = [0;0;0;0;1];
    C_aug = zeros(4,5);


    %load param
    Caf = param.Caf;
    Car = param.Car;
    m = param.m;
    lf = param.lf;
    lr = param.lr;
    Iz = param.Iz;

    Ac = [0    1                               vx    0; 
         0   -1*(2*Caf+2*Caf)/(m*vx)          0    -vx-1*(2*Caf*lf-2*Car*lr)/(m*vx);
         0    0                               0     1;
         0   -1*(2*Caf*lf-2*Car*lr)/(Iz*vx)   0    -1*(2*Caf*lf.^2 + 2*Car*lr.^2)/(Iz*vx)];

    Bc = [0;  2*Caf/m;   0;   2*Caf*lf/Iz];

    Cc = [1 0 0 0;
         0 1 0 0;
         0 0 1 0;
         0 0 0 1];
    

    % %%%%%%Discretization by Euler Method
    % n = size(Ac,2);
    % 
    % Ts = 0.01;
    % 
    % Ad = eye(n)+Ac*Ts;
    % Bd = Bc*Ts;
    % Cd = Cc*Ts;
    % 
    % 
    % % [Ad, Bd, Cd, ~] = c2d(A,B,C,D,dt);
    % A_aug(1:4,1:4) = Ad;
    % A_aug(1:4,5) = Bd;
    % A_aug(5,5) = 1;
    % 
    % B_aug(1:4,:) = Bd;
    % 
    % C_aug(1:4,1:4) = Cd;

    %%%%%%Discretization by Euler Method

    A_aug(1:4,1:4) = Ac;
    A_aug(1:4,5) = Bc;
    A_aug(5,5) = 1;

    B_aug(1:4,:) = Bc;

    C_aug(1:4,1:4) = Cc;

    n = size(A_aug,2);

    Ts = 0.01;
    A_aug_d = eye(n)+A_aug*Ts;
    B_aug_d = B_aug*Ts;
    C_aug_d = C_aug;




end
