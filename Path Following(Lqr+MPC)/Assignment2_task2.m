clc                            
close all                       
clear all                       
%% Parameters 

run ('num_parameters.m');        % Run script to set numerical parameters
run ('parameter_init.m');        % Run script to initialize parameters

%% LQR
xdotPP = @xdotPP;                % Define function handle for state-space system
%% Recalling function

Vx = velocity_interval(1);
Vdes = velocity_interval(2);

% Compute state matrix A
A = Amatrix(mass,Vx,cf,cr,lf,lr,Iz); 

% Compute input matrix B1
B = B1matrix(mass,cf,lf,Iz);   

% Compute disturbance input matrix B_d
B_d = B_dmatrix(mass,cf,cr,Iz,Vx,lf,lr); 

% Identity matrix for output
C = eye(4); 

% Zeros matrix for direct feedthrough
D = zeros(4,2);


%%  CONTROLLER 

% Initial state
x0 = [-2; 0; 0; 0];

% Default plot storage
t_plot = 0:dt:t_end*2;           % Simulation time
heading_plot = 0;                 % Vehicle heading
heading_error = 0;                % Vehicle heading error
slip_angle = 0;                   % Slip angle
course_angle = 0;                 % Course angle
steering_angle = 0;               % Steering angle
pos_des = [0; 0];                 % Desired position
position_global = [0; -2; 0];     % Global position
vel_plot = 0;
acceleration = 0;

% Check the reachability
% Controllability matrix
R = [B A*B (A^2)*B (A^3)*B];  

% Rank of controllability matrix
rank_R = rank(R);

if rank_R == length(A)
    disp('FULLY REACHABLE')
else 
    disp('NOT FULLY REACHABLE')
end

xpp= x0;                          % Initialize state variable
%% Controller simulation 

% Boolean parameter for controller update
boolean_param = false;           

% Get controller gains( This function needs a setup in case we are using
% it for the 2nd or the 3rd assignment)
K_update = K_update_vel_interval(velocity_interval, boolean_param); 

%define the values of the proportional and integral controlelrs 
[G, Kp, Ki] = PI_tune(tau, ratio, desired_settling_time, tolerance); 

while t <= 2*t_end
    % PI Controller

    % Compute velocity error integral
    integrator = Velocity_integrator(t,Vx,Vdes);

    % Update reference velocity
    longitudinal_ref = [longitudinal_ref integrator];

    % Simulate PI controller
    [tsol, x_pi] = ode45(@(t,longitudinal_ref) PI(t,Vx,Vdes,Kp,Ki), [t t+dt], longitudinal_ref(:,end)); 

    % Update longitudinal reference
    longitudinal_des= [longitudinal_des x_pi(end,:)]; 

    % Simulate PIC controller
    [tsol, x_pic] = ode45(@(t,longitudinal_des) PIC(t, tau, longitudinal_des), [t t+dt], longitudinal_des(:,end));

    % Update longitudinal position
    longitudinal_pos= [longitudinal_pos x_pic(end,:)]; 
  
    % Update velocity
    V_update = Vx;                 
    if t ~= 0
        % Update velocity based on position
        V_update = V_update + longitudinal_pos(end)/t; 
    end

    % Choose the appropriate control gain K based on the variation of Vx
    if (V_update * 3.6) >= 30 && (V_update * 3.6) < 40 - e_acc
        index = 1;
    elseif (V_update * 3.6) >= 40 && (V_update * 3.6) < 50 - e_acc
        index = 2;
    elseif (V_update * 3.6) >= 50 && (V_update * 3.6) < 60 - e_acc
        index = 3;
    elseif (V_update * 3.6) >= 60 && (V_update * 3.6) < 70 - e_acc
        index = 4;
    elseif (V_update * 3.6) >= 70 && (V_update * 3.6) < 80 - e_acc
        index = 5;
    elseif (V_update * 3.6) >= 80 && (V_update * 3.6) < 90 - e_acc
        index = 6;
    elseif (V_update * 3.6) >= 90 && (V_update * 3.6) < 100 - e_acc
        index = 7;
    elseif (V_update * 3.6) >= 100 && (V_update * 3.6) < 110 - e_acc
        index = 8;
    elseif (V_update * 3.6) >= 110 && (V_update * 3.6) < 120 - e_acc
        index = 9;
    else
        index = 10;
    end

    % Storing the values of the velocity
    vel_plot =[vel_plot (V_update*3.6)'];
    %acceleration = dv/dt
    acceleration = [acceleration (longitudinal_pos(end)/t)'];

    % Update state matrix A based on current velocity
    A1 = Amatrix(mass,V_update,cf,cr,lf,lr,Iz);   

    % Update input matrix B1 based on current velocity
    B1 = B1matrix(mass,cf,lf,Iz);         

    % Update disturbance input matrix B_d based on current velocity
    B_d1 = B_dmatrix(mass,cf,cr,Iz,V_update,lf,lr); 

    % Choose controller gains based on speed
    K = K_update{index};                       

    % Compute desired path parameters
    [w, psi_des_t, xdes_t, ydes_t] = Desired_path(V_update, r, t);

    % Define the feedforward term for zero steady-state error
    delta_ff = feedforward(mass, V_update, lf, lr, cf, cr, r, K);

    % Simulate system dynamics
    [tsol, xsolpp] = ode45(@(t,x) xdotPP(A1,B1,x,B_d1,w,K,delta_ff), [t t+dt], xpp(:,end)); 

    % Update state variable
    xpp = [xpp xsolpp(end,:)'];                
    % UPDATES 

    % Update heading 
    heading = xpp(3, end) + psi_des_t;

    % Store heading data
    heading_plot = [heading_plot xpp(3, end)'];

    % Update global position 
    % Update global X position
    X1 = xdes_t - xpp(1,end)*sin(heading);

    % Update global Y position
    Y1 = ydes_t + xpp(1,end)*cos(heading);

    % Store desired position data
    pos_des = [pos_des [xdes_t ydes_t]']; 

    % Store global position data
    position_global = [position_global [X1 Y1 heading]'];

    % Update slip angle
    vehicle_slip = (1/V_update)*xpp(2, end) - xpp(3, end); % Compute slip angle

    % Store slip angle data
    slip = [slip vehicle_slip']; 


    % Update course angle
    course_t = heading + vehicle_slip;  
    
    % Store course angle data
    course_angle = [course_angle course_t'];  

    % Update steering angle 
    u = (-K*xpp(:,end)) + delta_ff;   

    % Store steering angle data
    steering_angle = [steering_angle u']; 

    % Expected heading error
    e2 = (-(lr/r)) + ((lf/(2* cr * (lf + lr))) * ((mass*V_update^2)/r));

    % Store heading error data
    heading_error = [heading_error e2'];   

    % Increment time
    t = t + dt;                               
    if t >= 10

        % Radius for a circular path after 10 seconds
        r = 1000;   
    else

        % Infinite radius for a straight line for 10 seconds
        r = inf;   
    end  
end
%% Plot the result

% Re-fetch the time simulation
t_plot = 0:dt:2*t_end;

% Plot heading
f1=figure(1);
subplot(3,2,1);
plot(t_plot, heading_plot,'r','LineWidth', 1.5);
title('Heading');
xlabel('Time (s)');
ylabel('Heading (rad)');

% Plot heading error
subplot(3,2,2);
plot(t_plot, heading_error,'g','LineWidth', 1.5);
title('Heading Error');
xlabel('Time (s)');
ylabel('Heading Error (rad)');

% Plot slip angle
subplot(3,2,3);
plot(t_plot, slip,'b','LineWidth', 1.5);
title('Slip Angle');
xlabel('Time (s)');
ylabel('Slip Angle (rad)');

% Plot course angle
subplot(3,2,4);
plot(t_plot, course_angle,'c','LineWidth', 1.5);
title('Course Angle');
xlabel('Time (s)');
ylabel('Course Angle (rad)');

% Plot steering angle
subplot(3,2,5);
plot(t_plot, steering_angle,'m','LineWidth', 1.5);
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (rad)');

% Plot global position
subplot(3,2,6);
plot(position_global(1,:),position_global(2,:), 'b','LineWidth', 1.5);
hold on;
plot(pos_des(1,:), pos_des(2,:), 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on;

f2= figure(2);
plot(position_global(1,:),position_global(2,:), 'b','LineWidth', 1.5);
hold on;
plot(pos_des(1,:), pos_des(2,:), 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on

f3 = figure(3);
subplot(2,1,1);
plot(t_plot, vel_plot,'c','LineWidth', 1.5);
title('Longitudinal velocity');
xlabel('Time (s)');
ylabel('Longitudinal velocity');
grid on 

subplot(2,1,2)
plot(t_plot,acceleration,'r','LineWidth',1.5);
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration');
grid on 

