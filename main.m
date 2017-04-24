clc
clear all
close all
%x=[q1,q2,q3,q4,q5,dq1,dq2,dq3,dq4,dq5]
%x_0=rand(10,1);
x_0 = [0;0.2625;0.8010;0.0292;0;0;0.4886;0.5785;0.2373;0];
x_0(1)= 0;
x_0(5) = 0;
x_0(6) = 0;
x_0(10) = 0;


x_f=rand(10,1);
x_f = [0.6596;
    0.5186;
    
    0.9730;
    0.6490;
    0.8003;
    0.4538;
    0.4324;
    0.8253;
    0.0835;
    0.1332];
x_f(1)= 0;
x_f(5) = 0;
x_f(6) = 0;
x_f(10) = 0;
%robustControl(x_0,x_f)
tf = 2;

% Generate Trajectory for control
traj = [];
j = 1;
for i=1:5
    traj = [traj;generateTrajectory([x_0(j); x_0(j+5)],[x_f(j); x_f(j+5)], tf)'];
    j = j+1;
end

[torque, x] = passivity_normal(traj, tf, x_0);

for j = 1:size(x,1)
    ee_pose = forwardKin(x(j,1:5));
    controller_position(:,j) = ee_pose(1:3,4);
end

%% Generate Trajectories for Plot
coeff = [];
j = 1;
for i=1:5
    coeff = [coeff;generateTrajectory([x_0(j); x_0(j+1)],[x_f(j); x_f(j+5)], tf)'];
    j = j + 1;
end

theta_d=[];
for t=0:0.01:tf
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d= [theta_d, [coeff(1,:)*vec_t; coeff(2,:)*vec_t; coeff(3,:)*vec_t; coeff(4,:)*vec_t;coeff(5,:)*vec_t;]];
end

for j = 1:size(theta_d,2)
    ee_pose = forwardKin(theta_d(:,j));
    position(:,j) = ee_pose(1:3,4);
end
figure
az = 0;
el = 0;
plot3(position(1,:),position(2,:),position(3,:),'-r')
view(az,el);
grid on
title('Ideal Trajectory')

% Plot Controller Cartesian Trajectory
figure;
plot(controller_position(1,:),controller_position(3,:),'-r')
grid on
title('Controller Cartesian Trajectory')

%% Plot Torques
figure
plot(torque(1,:), 'LineWidth', 5);
title('Joint Torques')
hold on
grid on
plot(torque(2,:), 'LineWidth', 5);
plot(torque(3,:), 'LineWidth', 5);
plot(torque(4,:), 'LineWidth', 5);
plot(torque(5,:), 'LineWidth', 5);
legend('Link 1 Torque', 'Link 2 Torque', 'Link 3 Torque', 'Link 4 Torque', 'Link 5 Torque');

%%
t = linspace(1, 100, length(x))';
cjt = [t, x(:,2:4)]; %controlled joint trajectory
open_system('youBot'); %open simulation file
sim('youBot', 'StopTime', '100');

% trqe = [t', torque'];
%inverse_dynamics(traj, tf);
% % Generate Trajectories for Plot
% coeff = [];
% j = 1;
% for i=1:4
%     coeff = [coeff;generateTrajectory(x_0(j:j+1),x_f(j:j+1), tf)'];
%     j = j + 2;
% end
% 
% 
% 
% coeff
% theta_d=[];
% for t=0:0.01:tf
%     vec_t = [1; t; t^2; t^3]; % cubic polynomials
%     theta_d= [theta_d, [coeff(1,:)*vec_t; coeff(2,:)*vec_t; coeff(3,:)*vec_t; coeff(4,:)*vec_t;]];
% end
% 
% for j = 1:size(theta_d,2)
%     ee_pose = forwardKin([0;theta_d(:,j)]);
%     position(:,j) = ee_pose(1:3,4);
% end
% figure
% plot3(position(1,:),position(2,:),position(3,:),'-r')
% title('Ideal Trajectory')
% passivity_normal(traj, tf);