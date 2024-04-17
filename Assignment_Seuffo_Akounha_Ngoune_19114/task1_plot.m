clc
close all 
clear all 

% Nominal Parameters
% Velocity 
velocity = 30; % m/s
Vx = (velocity * 1000) / 3600; % m/s
% Steering angle 
amplitude = pi/6; % amplitude of the input 
amp1 = 30; %amplitude in radius 
frequency = 0.5; 
lf = 1.3384; % m 
lr = 1.6456; % m 
deltaf = @(t) deg2rad(amp1 * sin(2 * pi * frequency * t));
%Initialise parameters 
betav = 0;
BETAV = [betav'];
course_heading = 0;
course = [course_heading'];




% Simulation parameters
tsim = 10;
Tstep = 0.1;
t = 0;

% Initial conditions
x0 = [0 0 0]';
x = x0;

% Simulation loop
while t <= tsim
    u = [Vx, deltaf(t), 0];
    betav = atan((lr * tan(u(2))) / (lf + lr));
    [tsol, xsol] = ode45(@(t, x) task1_init(t, x, u), [t t + Tstep], x(:, end));
    x = [x xsol(end,:)'];
    BETAV = [BETAV betav'];
    course_heading = betav + x(3,end);
    course = [course course_heading'];
    t = t + Tstep;
end

tp = 0:Tstep:tsim+Tstep;
disp(length(tp));
disp(tp);

% Create a figure
figure;
% Plotting the vehicle position
subplot(3, 2, [1,2]);
plot(x(1, 1:end-1), x(2, 1:end-1), 'r-', 'LineWidth', 1.5);
title('Vehicle');
xlabel('X Position');
ylabel('Y Position');
grid on;


disp(length(tsol));
disp(length(x(3,1:end)));
% Plotting the vehicle heading
subplot(3, 2, 3);
plot(tp, x(3,1:end), 'b-', 'LineWidth', 1.5);
title('Vehicle Heading');
xlabel('Time (s)');
ylabel('Heading Angle (degrees)');
grid on;

% Plotting the slip angle
subplot(3, 2, 4);
plot(tp, BETAV(1,1:end), 'g-', 'LineWidth', 1.5);
title('Slip Angle');
xlabel('Time (s)');
ylabel('Slip Angle (degrees)');
grid on;



% Plotting the course heading 
subplot(3, 2, 5);
plot(tp, rad2deg(course(1,1:end)), 'y-', 'LineWidth', 1.5);
title('Course Heading');
xlabel('Time(s)');
ylabel('Course Heading(deg)');
grid on;









