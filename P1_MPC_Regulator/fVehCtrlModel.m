function [Ad, Bd, Cd] = fVehCtrlModel(vx, param)
    
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
    

    %%%%%%Discretization by Euler Method
    n = size(Ac,2);

    Ts = 0.01;

    Ad = eye(n)+Ac*Ts;
    Bd = Bc*Ts;
    Cd = Cc*Ts;


    % [Ad, Bd, Cd, ~] = c2d(A,B,C,D,dt);


end
