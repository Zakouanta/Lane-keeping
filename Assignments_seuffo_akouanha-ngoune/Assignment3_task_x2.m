clc
close all 
clear all 


%Load numerical parameters 
run('num_parameters.m');
run('parameter_init.m');

%Initial state
x0 = [1.5;0;0;0];

%Velocity 
velocity_interval = [20 30];
Vx = velocity_interval(1)/3.6;
Vdes = velocity_interval(2)/3.6;

%Setup for MPC 
[Ac,Bc,C,D,n,m] = computeMatrices(mass,Vdes,cf,cr,lf,lr,Iz);
sysc = ss(Ac, Bc, C, D);
T = 5; % Prediction time
delta = 0.092; % Sampling time for discretization
% Prediction horizon
N = fix(T/delta);
sysd = c2d(sysc, delta, 'zoh');
Ad = sysd.A;
Bd = sysd.B;

%setup for LQR 
A = Amatrix(mass,Vdes,cf,cr,lf,lr,Iz);% Compute state matrix A
B = B1matrix(mass,cf,lf,Iz); % Compute input matrix B1

% Gain matrix 
[Q_lqr, K_lqr, S, CLP] = LQR(A, B);   % Compute LQR gain

% Initial Conditions for Integration
xmeasure = x0; % Initial condition for MPC
xpp = x0; % Initial condition for LQR
%initial progression along the path
s_current = 0;

% Main simulation parameters
t_end = 10; % Simulation end time
dt = delta; % Simulation time step (should match MPC sampling time for consistency)
t = 0:dt:t_end;

% Preallocate storage for results for plotting 
heading_plot = 0;                 % Vehicle heading
heading_error = 0;                % Vehicle heading error
slip_angle = 0;                   % Slip angle
course_angle = 0;                 % Course angle
steering_angle = 0;               % Steering angle
pos_des = [0;0];                     % Desired position
position_global = [0;-2;0];            % Global position
vel_plot = 0;
acceleration = 0;

longitudinal_ref = []; % To store longitudinal reference over time
longitudinal_des = []; % To store desired longitudinal position over time
longitudinal_pos = []; % To store longitudinal position updates

%Gain parameters 
boolean_param = false;% Boolean parameter for controller update  
% Get controller gains
K_update = K_update_vel_interval(velocity_interval, boolean_param); 



%desired path
[x_des_mpc, y_des_mpc, psidot_des_mpc, psi_des_mpc, s_ref] = reference_mpc_left_turn(Vdes, ...
                                                                            L_straight, L_straight, ...
                                                                            N_points);
% MPC Path Planning
[xref,~,~,global_X,global_Y] = SolveMPC_left_turn(Ad,Bd,n,m, ...
                                                x0,N,x_des_mpc, ...
                                                y_des_mpc, psidot_des_mpc, ...
                                                psi_des_mpc, ...
                                                s_ref,Vdes); 

%define the values of the proportional and integral controlelrs 
[G, Kp, Ki] = PI_tune(tau, ratio, desired_settling_time, tolerance); 
% Run simulation
for k = 1:length(t)


    % PI Controller

    % Compute velocity error integral
    integrator = Velocity_integrator(t(k),Vx,Vdes);

    % Update reference velocity
    longitudinal_ref = [longitudinal_ref integrator];

    % Simulate PI controller
    [tsol, x_pi] = ode45(@(t,longitudinal_ref) PI(t,Vx,Vdes,Kp,Ki), ...
                                                 [t(k),t(k)+dt], ...
                                                 longitudinal_ref(:,end)); 

    % Update longitudinal reference
    longitudinal_des= [longitudinal_des x_pi(end,:)]; 

    % Simulate PIC controller
    [tsol, x_pic] = ode45(@(t,longitudinal_des) PIC(t, tau, longitudinal_des), ...
                                                   [t(k), t(k)+dt], ...
                                                   longitudinal_des(:,end));

    % Update longitudinal position
    longitudinal_pos= [longitudinal_pos x_pic(end,:)]; 
  
    % Update velocity
    V_update = Vx;                 
    if t(k) ~= 0
        % Update velocity based on position
        V_update = V_update + longitudinal_pos(end)/t(k); 
    end

    % Choose the appropriate control gain K based on the variation of Vx
    if (V_update * 3.6) >= 20 && (V_update * 3.6) < 25 - e_acc
        index = 1;
    elseif (V_update * 3.6) >= 25 && (V_update * 3.6) < 30 - e_acc
        index = 2;
    else
        index = 3;
    end

    % Storing the values of the velocity
    vel_plot =[vel_plot (V_update*3.6)'];
    %acceleration = dv/dt
    acceleration = [acceleration (longitudinal_pos(end)/t(k))'];

    %desired path
    [ ~, ~, psidot_des_mpc, psi_des_mpc, s_ref] = reference_mpc_left_turn(V_update, ...
                                                                    R_mpc, L_straight, ...
                                                                    N_points);
    % Update s_current based on Vx and tmeasure
    % For a more accurate simulation, this might involve integrating over the actual path traveled, considering dynamics.
    s_current = s_current + V_update * t(k);
    
    % Ensure s_current does not exceed the total path length
    s_current = min(s_current, s_ref(end));
    
    % Find the closest index in s_ref to the current s_current
    [~, idx] = min(abs(s_ref - s_current));
    
    % Use the psidot_des_mpc and psi_des value at this index for the vehicle dynamics update
    psidot_des_value = psidot_des_mpc(idx);
    psides_value = psi_des_mpc(idx);


    % Update state matrix A based on current velocity
    A1 = Amatrix(mass,V_update,cf,cr,lf,lr,Iz);   

    % Update input matrix B1 based on current velocity
    B1 = B1matrix(mass,cf,lf,Iz);         

    % Update disturbance input matrix B_d based on current velocity
    B_d1 = B_dmatrix(mass,cf,cr,Iz,V_update,lf,lr); 

    % Choose controller gains based on speed
    K = K_update{index}; 
    
    % Adapt the cost parameters 
    [Q, R, Pf] = adaptCostParameters(V_update, Vx);
    
    % Calculate Deviation between the optimisation result and the Lqr
    state_error = xref - xmeasure; 

    % Define the feedforward term for zero steady-state error
    delta_ff = feedforward(mass, V_update, lf, lr, cf, cr, r, K);
    
    % Combine LQR correction with feedforward term 
    u_lqr = K * state_error + delta_ff;

    % Simulate system dynamics using ode45, incorporating the new control input
    [tsol, x_sol] = ode45(@(t,x) xdotPP(A1,B1,x,B_d1,psidot_des_value,K,delta_ff), [t(k) t(k)+dt], xmeasure); 
    
    % Update state variable
    xpp = [xpp x_sol(end,:)'];
                                     
    % Update the sate variable base on the new control input 
    xmeasure = xpp(:,end);

    % UPDATES 

    % Update heading 
    heading = xpp(3, end) + psides_value;

    % Store heading data
    heading_plot = [heading_plot xpp(3, end)'];

    % Update global position 
    % Update global X position
    X1 = global_X - xpp(1,end)*sin(heading);

    % Update global Y position
    Y1 = global_Y + xpp(1,end)*cos(heading);

    % Store desired position data
    % pos_des = [pos_des [global_X global_Y]']; 

    % Store the steering angle 
    u = (-K*xpp(:,end)) + delta_ff;

    % Update slip angle
    vehicle_slip = (1/V_update)*xpp(2, end) - xpp(3, end); % Compute slip angle

    % Store slip angle data
    slip = [slip vehicle_slip']; 

    % Update course angle
    course_t = heading + vehicle_slip;  
    
    % Store course angle data
    course_angle = [course_angle course_t'];    

    % Store steering angle data
    steering_angle = [steering_angle u']; 

    % Expected heading error
    e2 = (-(lr/r)) + ((lf/(2* cr * (lf + lr))) * ((mass*V_update^2)/(Vx/psidot_des_value)));

    % Store heading error data
    heading_error = [heading_error e2']; 

    % Update Time 
    t = t + dt;
end

% Time vector
t = 0:dt:(length(heading_plot)-1)*dt;

% Plot heading
f1=figure(1);
subplot(3,2,1);
plot(t, heading_plot,'r','LineWidth', 1.5);
title('Heading');
xlabel('Time (s)');
ylabel('Heading (rad)');

% Plot heading error
subplot(3,2,2);
plot(t, heading_error,'g','LineWidth', 1.5);
title('Heading Error');
xlabel('Time (s)');
ylabel('Heading Error (rad)');

% Plot slip angle
subplot(3,2,3);
plot(t, slip,'b','LineWidth', 1.5);
title('Slip Angle');
xlabel('Time (s)');
ylabel('Slip Angle (rad)');

% Plot course angle
subplot(3,2,4);
plot(t, course_angle,'c','LineWidth', 1.5);
title('Course Angle');
xlabel('Time (s)');
ylabel('Course Angle (rad)');

% Plot steering angle
subplot(3,2,5);
plot(t, steering_angle,'m','LineWidth', 1.5);
title('Steering Angle');
xlabel('Time (s)');
ylabel('Steering Angle (rad)');

% Plot global position
subplot(3,2,6);
plot(X1, Y1, 'b','LineWidth', 1.5);
hold on;
plot(global_X, global_Y, 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on;

% Plot global position
subplot(3,2,6);
plot(X1, Y1, 'b','LineWidth', 1.5);
hold on;
plot(global_X, global_Y, 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on;

f2= figure(2);
plot(X1, Y1, 'b','LineWidth', 1.5);
hold on;
plot(global_X, global_Y, 'r--','LineWidth', 1.5);
title('Global Position');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Actual Position', 'Desired Position');
grid on;

f3 = figure(3);
subplot(2,1,1);
plot(t, vel_plot,'c','LineWidth', 1.5);
title('Longitudinal velocity');
xlabel('Time (s)');
ylabel('Longitudinal velocity');
grid on 

subplot(2,1,2)
plot(t,acceleration,'r','LineWidth',1.5);
title('Acceleration');
xlabel('Time (s)');
ylabel('Acceleration');
grid on 