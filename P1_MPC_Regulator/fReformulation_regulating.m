function [bar_H, bar_F,A_bar,Q_bar,Q] = fReformulation_regulating(Ad,Bd,mpcparam)
    %step 1. C_bar, R_bar, Q_bar, A_bar 구하기
    nx = size(Ad,2);  %number of state
    nu = size(Bd,2);  %number of input
    Q = mpcparam.Q; % State weighting matrix
    R = mpcparam.R; % Control weighting matrix
    S = mpcparam.S; % Terminal cost weighting matrix
    Np = mpcparam.Np; % Prediction horizon (assume that Nc=Np)
    C_bar = zeros(Np*nx,Np*nu);
    A_bar = zeros(Np*nx,nx);
    Q_bar = eye(4*Np,4*Np);
    R_bar = eye(Np);

    %R_bar
    R_bar = R*R_bar;

    %Q_bar
    for i=1:Np           %행
      for j=1:Np         %열
          if i==j
              Q_bar(4*i-3:i*4,4*j-3:j*4)=Q;
          else
              Q_bar(4*i-3:i*4,4*j-3:j*4)=zeros(4,4);
          end
      end
    end

    %Terminal Cost Tuning
    Q_bar(4*Np-3:4*Np,4*Np-3:4*Np)=S;
    
    %A_bar
    for i=1:Np
        A_bar((i-1)*nx+1:i*nx,:)=Ad^(i);
    end
    
    %C_bar
    for i=1:Np+1         %행
      for j=1:Np         %열
          if i>j
              C_bar((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=Ad^(i-j-1)*Bd;
          else
              C_bar((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=zeros(nx,nu);
          end    
      end
    end

    C_bar(1:nx,:)=[];
    


    %step 2. H, F 구하기
    %%%%%%%% H = C_bar의 Transpose*Q_bar*C_bar + R_bar
    %%%%%%%% F = C_bar의 Transpose*Q_bar*A_bar

    % H
    bar_H=C_bar.'*Q_bar*C_bar + R_bar;

    % F
    bar_F = C_bar.'*Q_bar*A_bar;

end