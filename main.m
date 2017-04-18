clc
clear all
close all
%x=[q1,q2,q3,q4,dq1,dq2,dq3,dq4]
x_0=rand(8,1);
x_f=rand(8,1);
%robustControl(x_0,x_f)
tf = 2;

% Generate Trajectories
coeff = [];
j = 1;
for i=1:4
    coeff = [coeff;generateTrajectory(x_0(j:j+1),x_f(j:j+1), tf)'];
    j = j + 2;
end
theta_d=[];
for t=0:0.01:tf
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d= [theta_d, [coeff(1,:)*vec_t; coeff(2,:)*vec_t; coeff(3,:)*vec_t; coeff(4,:)*vec_t;]];
end

for j = 1:size(theta_d,2)
    ee_pose = forwardKin([0;theta_d(:,j)]);
    position(:,j) = ee_pose(1:3,4);
end
    % passivity_normal(traj, tf);