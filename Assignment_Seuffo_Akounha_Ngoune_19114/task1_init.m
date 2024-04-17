function xdot = task1_init(t,x,u)
%% Definig Parameters 
lf = 1.3384 ;% m 
lr =  1.6456; % m   
% Defining the sinusoidal steering input 
betav = atan((lr*tan(u(3)))/(lf + lr));
xdot = [u(1)*cos(x(3) + betav) u(1)*sin(x(3) + betav) (u(1)*cos(betav)* tan(u(2)))/(lr+lf)]';
 