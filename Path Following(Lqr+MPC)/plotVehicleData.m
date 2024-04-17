function plotVehicleData(t_plot, heading_plot,heading_error,slip,course_angle,steering_angle,position_global,pos_des)
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
end