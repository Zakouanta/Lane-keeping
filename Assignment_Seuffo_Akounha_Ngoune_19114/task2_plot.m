clc 
clear all 
close all 

%% Parameters

m = 2164; %kg 
Iz = 4373 ; % Kg.m^2
velocity = 70; % m/s
Vx = (velocity*1000)/3600; % m/s
amplitude = pi/6; % amplitude of the input 
frequency = 0.5; 
lf = 1.3384; % m 
lr = 1.6456; % m 
L= lf+lr;

cf = 1.0745*10^5;
cr = 1.9032*10^5;
r=1000; % Radius of the circular desired path 
deltap = 0.6; 
zeta = 5; % settling time
omega = deltap*zeta;
 



%% Computation 

% We just need the initial condition condition to be close enough to zero
% so we define a random numbers with equal prob between -1 and 1 
% x_0 = 2*rand(4,1)-1;
x_0 = zeros(4,1);

% Compute matrices 
A=Amatrix(m,Vx,cf,cr,lf,lr,Iz);
B = Bmatrix(m,cf,Iz);
C = eye(4);
D = zeros(4,1);
u = deltaf(amplitude,frequency);

%Compute matrices for the road aligned model 

A1 = A1matrix(m,Vx,cf,cr,lf,lr,Iz);
B1_1 = B1_1matrix(m,cf,lf,Iz);
B_d = B_dmatrix(m,cf,cr,Iz,Vx,lf,lr);
B1 = [B1_1 B_d];
C1 = eye(4);
D1 = zeros(4,2);
%Initial states 
x2_0 = zeros (4,1)+2;
% psi_des 
psidot_des = Vx/r;
%% State Feedback 

%Let us write the reachability matrix first 
% The rank of the A matrix is 4 so we write the reachability matrix until
% 4-1=3

R = [B1_1 A1*B1_1 (A1^2)*B1_1 (A1^3)*B1_1];
rank_R = rank(R);

if rank_R == length(A1)
    disp('FULLY REACHABLE')
else 
    disp('NOT FULLY REACHABLE')
end

% Desired position of the states, the moltiplicity of a pole must be
% greater than rank(B1_1)
p1 = -deltap*omega+1i*omega*sqrt(1-deltap^2);
p2= -deltap*omega-1i*omega*sqrt(1-deltap^2);

P = [p1 p2 -30  -80];



% Step 2:determining the gain matrix 
Kpp = place(A1,B1_1,P);
% Defining the new A matrix just for testing reason 
App = A1-B1_1*Kpp; 


%Defining the feed forward term for a zero ss error 

delta_ff = feedforward(m, Vx, lf, lr, cf, cr, r, Kpp);


%% LQR

%Q and R matrices: We penalize bad performances buy adjusting Q and we
%penalize actuator effort wit R.

Q = 0.01;
R_lqr =1;

K_lqr = lqr(A1,B1_1,Q,R_lqr);




%% Plot 







