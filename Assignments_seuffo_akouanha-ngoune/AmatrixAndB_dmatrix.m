function [A_update, B_d_update] = AmatrixAndB_dmatrix(m, Vx, cf, cr, lf, lr, Iz)
    A_update = [];  % Initialize A_update
    B_d_update = [];  % Initialize B_d_update
    while Vx >= 30/3.6 && Vx <= 120/3.6
        A = [0 1 0 0; 
             0 -((2*cf + 2*cr)/(m*Vx)) ((2*cf + 2*cr)/(m)) (((-2*cf*lf) + 2*cr*lr)/(m*Vx));
             0 0 0 1;
             0 -((2*cf*lf-2*cr*lr)/(Iz*Vx)) ((2*cf*lf-2*cr*lr)/Iz) -((2*cf*(lf^2)+(2*cr*(lr^2)))/(Iz*Vx))];
        
        B_d = [0; ((-((2*cf*lf - 2*cr*lr)/(m*Vx)))-Vx); 0; -((2*cf*(lf^2)+2*cr*(lr^2))/(Iz*Vx))];
        
        A_update = [A_update A];
        B_d_update = [B_d_update B_d];

        Vx = Vx + 10/3.6; 

        % Update Vx within the specified interval for the next iteration
        Vx = max(30/3.6, min(120/3.6, Vx));
    end
end