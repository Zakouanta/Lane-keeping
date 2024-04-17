%% Init
e1 = 0;
e1_dot = 0;
e2 = -2;
e2_dot = 0;
xpp =[e1; e1_dot; e2; e2_dot];
w = 0;         
%  Position in the global framee
X1 = 0; % Coordinate X in the global frame
Y1 = 0; % Coordinate Y in the global frame
pos = [X1; Y1];
desired_position = [X1; Y1];               % Desire position (X,Y)
global_frame_pos = [X1; Y1];          % Global frame position (X,Y)
%vehicle parameters 
u=0;
heading = 0;       % Vehicle heading
slip = 0;       % Slip angle
x0 = [-2; 0; 0; 0];
heading_plot = 0;                % Vehicle heading
heading_error = 0;            % Vehicle heading error
slip_angle = 0;             % Slip angle
course_angle = 0;           % Course angle
steering_angle = u;         % Steering angle
pos_des = [0; 0];           % Desire position
position_global = [0; -2; 0];    % Global position
pos_tol = 0.3;


%% PI parameters init 
velocity_interval = [30/3.6 120/3.6]; %velocity interval 
longitudinal_ref= [];
longitudinal_des=[];
longitudinal_pos=[];
velocity_interval_mpc = [30/3.6 90/3.6]; %velocity interval 

