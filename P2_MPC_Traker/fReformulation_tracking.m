function [bar_H, transpose_bar_F] = fReformulation_regulating(A_aug,B_aug,C_aug,mpcparam)
    %step 1. C_bar, R_bar, Q_bar, A_bar 구하기
    nx = size(A_aug,2);  %number of state=5
    nu = size(B_aug,2);  %number of input=1
    Q = mpcparam.Q; % State weighting matrix
    R = mpcparam.R; % Control weighting matrix
    % S = mpcparam.S; % Terminal cost weighting matrix
    Np = mpcparam.Np; % Prediction horizon (assume that Nc=Np)
    C_doublebar = zeros(Np*nx,Np*nu);
    A_doublehat = zeros(Np*nx,nx);
    Q_doublebar = eye(5*Np,5*Np);
    T_doublebar = eye(4*Np,5*Np);
    R_bar = eye(Np);

    %R_bar
    R_bar = R*R_bar;

    %Q_doublebar
    for i=1:Np           %행
      for j=1:Np         %열
          if i==j
              Q_doublebar(5*i-4:i*5,5*j-4:j*5)=C_aug.'*Q*C_aug;
          else
              Q_doublebar(5*i-4:i*5,5*j-4:j*5)=zeros(5,5);
          end
      end
    end
    
    %T_doublebar
    for i=1:Np           %행
      for j=1:Np         %열
          if i==j
              T_doublebar(4*i-3:i*4,5*j-4:j*5)=Q*C_aug;
          else
              T_doublebar(4*i-3:i*4,5*j-4:j*5)=zeros(4,5);
          end
      end
    end


    %A_doublehat
    for i=1:Np
        A_doublehat((i-1)*nx+1:i*nx,:)=A_aug^(i);
    end

    %C_doublebar
    for i=1:Np+1         %행
      for j=1:Np         %열
          if i>j
              C_doublebar((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=A_aug^(i-j-1)*B_aug;
          else
              C_doublebar((i-1)*nx+1:i*nx,(j-1)*nu+1:j*nu)=zeros(nx,nu);
          end    
      end
    end

    C_doublebar(1:nx,:)=[];


    %step 2. H, F 구하기
    %%%%%%%% H = C_bar의 Transpose*Q_bar*C_bar + R_bar
    %%%%%%%% F = C_bar의 Transpose*Q_bar*A_bar

    % double bar H
    bar_H=C_doublebar'*Q_doublebar*C_doublebar + R_bar;

    % inverse double bar F
    transpose_bar_F = [A_doublehat'*Q_doublebar*C_doublebar;
                     -T_doublebar*C_doublebar];



end