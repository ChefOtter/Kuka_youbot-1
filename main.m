clc
clear all
close all
x_0=rand(8,1);
x_f=rand(8,1);
%robustControl(x_0,x_f)
tf = 2;

% Generate Trajectories
traj = [];
j = 1;
for i=1:4
    traj = [traj;generateTrajectory(x_0(j:j+1),x_f(j:j+1), tf)];
    j = j + 2;
end

passivity_normal(traj, tf);