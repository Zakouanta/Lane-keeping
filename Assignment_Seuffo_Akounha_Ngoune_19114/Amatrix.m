function A = Amatrix(m,Vx,cf,cr,lf,lr,Iz)

A= [0 1 0 0;
    0 -(2*cf+2*cr)/(m*Vx) 0 -Vx-(2*cf*lf-2*cr*lr)/(m*Vx);
    0 0 0 1;
    0 -(2*cf*lf - 2*cr*lr)/(Iz*Vx) 0 -(2*cf*lf^2 + 2*cr*lr^2)/(Iz*Vx)];

end