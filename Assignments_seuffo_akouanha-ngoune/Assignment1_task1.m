clc 
clear all 
close all 
%% Numerical parameters 
run num_parameters.m ;
%% Parammeters intialisation 
%  State vectors
e1 = 0;
e1_dot = 0;
e2 = 0;
e2_dot = 0;
x =[e1; e1_dot; e2; e2_dot];
w = 0;         
%  Position in the global framee
X1 = 0; % Coordinate X in the global frame
Y1 = 0; % Coordinate Y in the global frame
pos_des = [X1; Y1]; % Desire position (X,Y)
position_global = [X1; Y1];          % Global frame position (X,Y)
u = 0;
%vehicle parameters 
heading = 0;% Vehicle heading
steering_angle = 0;% Steering angle
slip = 0;       % Slip angle
course_angle = 0;       % Course angle
control_input = @(t) deg2rad(30 * sin(2 * pi * frequency * t));
%% computes matrices 
%Compute matrices for the road aligned model 
A = Amatrix(m,Vx,cf,cr,lf,lr,Iz);
B = B1matrix(m,cf,lf,Iz);
B_d = B_dmatrix(m,cf,cr,Iz,Vx,lf,lr);
C = eye(4);
D = zeros(4,2);
xdot = @xdot;
% psi_des 
%% Model Simulation 
while t<=t_end
    % Update u 
    u = control_input(t);
    [w, psi_des_t, x_des_t, y_des_t] = Desired_path(Vx, r, t);
    [tsol, xsol] = ode45(@(t,x) xdot(A,B,x,u,B_d,w), [t t+dt], x(:,end));
    x = [x xsol(end,:)'];  
    % Update the desired position
    pos_desired = [x_des_t y_des_t];
    pos_des = [pos_des pos_desired'];
    % Update the global position
    X = x_des_t - x(1,end)*sin(psi_des_t);
    Y = y_des_t + x(1,end)*cos(psi_des_t);
    global_pos = [X Y];
    position_global = [position_global global_pos'];
    % Update the vehicle heading
    psi = x(3, end) + psi_des_t;
    heading = [heading psi'];
    % Update the slip angle
    vehicle_slip = (1/Vx)*x(2, end) - x(3, end);
    slip = [slip vehicle_slip'];
    % Update the course angle
    course = psi + vehicle_slip;
    course_angle = [course_angle course'];
    % Update steering angle
    steering_angle = [steering_angle u'];
    t = t + dt;
    
end
%% Plot the result
%  Re-fetch the time simulation
t_plot = 0:dt:t_end+dt;
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