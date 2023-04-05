clc
clear

global x1 y1 x2 y2 x3 y3 x4 y4 theta th2

theta = 0 :0.1 : 2*pi;   % make circle
body_mass = 10;          %[kg]
wheel_mass = 3;          %[kg]
motor_mass = 4;          %[kg]
wheel_rad = 0.5;         %[m] 
motor_rad = 0.1;         %[kg]
g = 9.81;                %[m/s^2]

% axis offset--> each 10 --> 1m
wheel_center = [x1 y1];
end_of_body   = [x2 y2];
motor_center  = [x3 y3];

%time varient
dt = 0.01;
end_time = 10;

% Initial plot
FG = figure('color',[1 1 1],'Position',[800 300 800 500]);
AX = axes('parent', FG);
hold on;
grid on;
axis([-10 10  -0.5 10])

%set initial position
x1 = 0;   y1 = 0;      %wheel center 
x2 = 0;   y2 = 5;      %body end
x3 = x1;    y3 = y1;   %

x_wheel_coord = x1 + wheel_rad * cos(theta);
y_wheel_coord = y1 + wheel_rad * sin(theta);

x_motor_coord = x3 + motor_rad * cos(theta);
y_motor_coord = y3 + motor_rad * sin(theta);

plot(x_wheel_coord, y_wheel_coord,'lineWidth', wheel_mass);
plot(x_motor_coord, y_motor_coord, 'LineWidth',motor_mass)
line(wheel_center, end_of_body,'lineWidth', body_mass);




% x1, y1 is circle's center
% x2, y2 is end of body

xlabel('X Position','FontSize',20);
ylabel('Y position','FontSize',20);
title('self balancing(No Arm)');
grid on