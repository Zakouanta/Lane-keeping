function B_d = B_dmatrix(m,cf,cr,Iz,Vx,lf,lr)
 
    B_d = [0; ((-((2*cf*lf - 2*cr*lr)/(m*Vx)))-Vx); 0; -((2*cf*(lf^2)+2*cr*(lr^2))/(Iz*Vx))];
end 
