clc 
close all 
clear all 


%% Nominal Parameters 

syms m Vx Iz lf lr cf cr amplitude real

% m: mass of the vehicle [kg]
% Vx: longitudinal velocity expressed in the vehicle ref frame [m/s]
% Iz: total inertia vehicle around the center of gravity [kg m^2]
% lf, lr: distance of the front and read axles from the CG [m]
% cf, cr: cornering stiffness of the front and rear tires [N/rad]




%% System orientation 

%States
syms y y_dot psi psi_dot real
% y: lateral velocity in the vehicle's ref frame [m/s]
% psi_dot: yaw rate [rad]
% psi : heading of the vehicle 
% y_dot: lateral velocity 



%Control Input
deltaf = @(t) deg2rad(amplitude * sin(2 * pi * frequency * t));
% deltaf: front wheel steering angle [rad]

x = [y; y_dot; psi; psi_dot];
u = deltaf;


%% Computation

A= [0 1 0 0;
    0 -(2*cf+2*cr)/(m*Vx) 0 -Vx-(2*cf*lf - 2*cr*lr)/(m*Vx);
    0 0 0 1;
    0 -(2*cf*lf - 2*cr*lr)/(Iz*Vx) 0 -(2*cf*lf^2 + 2*cr*lr^2)/(Iz*Vx)];
B = [0;
     2*cf/m;
     0;
     2*cf/Iz];

C = eye (4);
D = zeros(4,4);





%% ROAD ALIGNED MODEL 


%Here we have the same nominal parameters, control states. We introduce the
%exogenous factor and the desired path which for this case is circle of
%radius R 



%Exogenous disturbance signals
syms psidot_des real 
%states 
syms e1 e1_dot e2 e2_dot real 

x1 =[e1; 
    e1_dot; 
    e2; 
    e2_dot];
w = psidot_des;
% matrices computation 
A1 = [0 1 0 0; 
    0 -((2*cf + 2*cr)/(m*Vx)) ((2*cf + 2*cr)/(m)) (((-2*cf*lf) + 2*cr*lr)/(m*Vx));
    0 0 0 1;
    0 -((2*cf*lf-2*cr*lr)/(Iz*Vx)) ((2*cf*lf-2*cr*lr)/Iz) -((2*cf*(lf^2)+(2*cr*(lr^2)))/(Iz*Vx))];
B1_1 = [0;(2*cf)/m; 0; (2*lf*cf)/Iz];

B_d = [0; ((-((2*cf*lf - 2*cr*lr)/(m*Vx)))-Vx); 0; -((2*cf*(lf^2)+2*cr*(lr^2))/(Iz*Vx))];

B1 = [B1_1 B_d];


C1 = eye (4);

D1 = zeros(4,2);







