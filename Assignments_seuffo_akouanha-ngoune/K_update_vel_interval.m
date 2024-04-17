function design_K = K_update_vel_interval(velocity_interval, boolean_param)
% loading Parameters 
run ('num_parameters.m');

deltap = 0.7;%damping ratio
d_Vx=10; %Velocity step (set to 5 for MPC) 

%velocity_interval = [20 30]; % Velocity interval for left turn 

%velocity_interval = [50 90];%Velocity interval for MPC 
V_min= velocity_interval(1)*1000/3600;
V_max= velocity_interval(2)*1000/3600;

%Recalling matrices for the min velocity 
A_min = Amatrix(mass,V_min,cf,cr,lf,lr,Iz);
B = B1matrix(mass,cf,lf,Iz);

%Recalling the matrices for the maximun value of Vx
A_max = Amatrix(mass,V_max,cf,cr,lf,lr,Iz);

[omega_n, ~] = specifications(deltap,deviation,zeta);
if boolean_param == true 
    K_min = PolePlacement(deltap,omega_n,A_min,B);
    K_max = PolePlacement(deltap,omega_n,A_max,B);
else 
    [Q_lqr, K_min, S, CLP] = LQR(A_min, B);
    [Q_lqr, K_max, S, CLP] = LQR(A_max, B);
end

%Initialiase and aray to store the values of K and set it to the the min if
%we get to the last one 
design_K = {};
design_K{end+1}= K_min;

% Set to 5 for MPC
for Vx = velocity_interval(1)+10:d_Vx:velocity_interval(2)-1

    Vx = Vx * 1000/3600;
    A = Amatrix(mass,Vx,cf,cr,lf,lr,Iz);

    if boolean_param == true 
        K_update = PolePlacement(deltap,omega_n,A,B);
    else 
       [~, K_update, ~, ~] = LQR(A, B);

    end 
    design_K{end+1} = K_update;
 end

design_K{end+1} = K_max;

end 