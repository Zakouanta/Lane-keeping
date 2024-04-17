function [A, B,C,D,n,m] = computeMatrices(m, Vx, cf, cr, lf, lr, Iz)
    % Compute state matrix A
    A = [0 1 0 0; 
         0 -((2*cf + 2*cr)/(m*Vx)) ((2*cf + 2*cr)/(m)) (((-2*cf*lf) + 2*cr*lr)/(m*Vx));
         0 0 0 1;
         0 -((2*cf*lf-2*cr*lr)/(Iz*Vx)) ((2*cf*lf-2*cr*lr)/Iz) -((2*cf*(lf^2)+(2*cr*(lr^2)))/(Iz*Vx))];

    % Compute input matrix B1
    B1 = [0;(2*cf)/m; 0; (2*lf*cf)/Iz];

    % Compute disturbance input matrix B_d
    B_d = [0; ((-((2*cf*lf - 2*cr*lr)/(m*Vx)))-Vx); 0; -((2*cf*(lf^2)+2*cr*(lr^2))/(Iz*Vx))];

    % Concatenate B1 and B_d
    B = [B1, B_d];

    % state dimension
    n = length(A(1,:)); 

    % input dimension
    m = 1; 

    % Identity matrix for output
    C = eye(n);
    
    % Zeros matrix for direct feedthrough
    D = zeros(n,2);
end
