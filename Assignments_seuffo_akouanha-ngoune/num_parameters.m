%% Simulation clock 
t = 0;          % Simulation time start
t_end = 10; % Simulation time end 
dt = 0.1;       % Simulation time step
t_pos = 0;
%% Simulation parameters
mass = 2164; %kg 
Iz = 4373 ; % Kg.m^2
velocity = 40; % m/s
Vx = (velocity*1000)/3600; % m/s
amplitude = pi/6; % amplitude of the input 
frequency = 0.5; 
lf = 1.3384; % m 
lr = 1.6456; % m 
L= lf+lr;   %Length of the vehiclle 
cf = 1.0745*10^5;% Cornering stiffness 
cr = 1.9032*10^5;
r=1000; % Radius of the circular desired path 
deltap = 0.7; 
zeta = 5; % settling time
%omega = deltap*zeta;
deviation = 30; 
%% Simulation Parameters for longitudinal control 
velocity_interval = [30 120]; %velocity interval
PI_interval = 0:0.01:0.75; % Interval for Kp and Ki
tau = 0.5;
ratio= 0.25;
desired_settling_time = 5;
tolerance = 20;
% Vdes = Vx + (50*1000)/3600;
e_acc= 1;
%% MPC parameters 
%Desired path 
[V_min, V_max, R_mpc, L_straight, N_points] = deal(30, 90, 1000, 200,3000);

% State cost matrix parameter 
param_q= 1; 

% Input cost matrix parameter 
param_r= 1e4; 

% Terminal state cost matrix parameter 
param_pf=1; 

% Upper bound e1 = 10 cm 
max_e1 = 0.1;
max_e11 = 0.1;

max_e1_final = 0.1;
max_e1_initial = 4;

% Upper bound e2
max_e2= 0.1;  
max_e2_final = 0.1;
max_e2_initial = pi;

%Lower bound on e1
%min_e1 = 0;

%Lower bound on e2
%min_e2 = 0.1; 

max_delta_u = pi; % Constraints on the steering angle 
min_distance = 20;



