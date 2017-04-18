function [] = passivity_normal(trajectory, tf )
%Passivity Based Control for Ideal Dynamics
% tau = M*a + C*v + N - Kr
% r = de + lambda*e
% v = dq - lambda*e
% a = ddq = lambda*de

%theta = [q1, q2, q3, q4, dq1, dq2, dq3, dq4]

torque = [];

options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x)planarArmODEUncertain(t,x),[0 tf],x0e,options);

function [dx ] = planarArmODEUncertain(t,x)
    K = 10*eye(4);
    lambda = 10*eye(4);
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d= [trajectory(1)'*vec_t; trajectory(2)'*vec_t; trajectory(3)'*vec_t; trajectory(4)'*vec_t;];
    
    % Joint 1 Velcoity and Acceleration
    a1_vel = [trajectory(1,2), 2*trajectory(1,3), 3*trajectory(1,4), 0];
    a1_acc = [2*trajectory(1,3), 6*trajectory(1,4),0,0 ];
    
    % Joint 2 Velcoity and Acceleration
    a2_vel = [trajectory(2,2), 2*trajectory(2,3), 3*trajectory(2,4), 0];
    a2_acc = [2*trajectory(2,3), 6*trajectory(2,4),0,0 ];
    
    % Joint 3 Velcoity and Acceleration
    a3_vel = [trajectory(3,2), 2*trajectory(3,3), 3*trajectory(3,4), 0];
    a3_acc = [2*trajectory(3,3), 6*trajectory(3,4),0,0 ];
    
    % Joint 4 Velcoity and Acceleration
    a4_vel = [trajectory(4,2), 2*trajectory(4,3), 3*trajectory(4,4), 0];
    a4_acc = [2*trajectory(4,3), 6*trajectory(4,4),0,0 ];
    
    % Desired Velocities
    dtheta_d =[a1_vel*vec_t; a2_vel* vec_t; a3_vel* vec_t; a4_vel* vec_t];
    
    %Desired Accelerations
    ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t; a3_acc* vec_t; a4_acc* vec_t];
    
    %Actual Velocities and Acceleration
    theta= x(1:4,1);
    dtheta= x(4:8,1);
    
    %Get Errors
    e = theta - theta_d;
    de = dtheta - dtheta_d;
    error = [e;de];
    
    %Get Passivity Parameters
    r = de + lambda*e;
    v = dtheta_d - lambda*e;
    a = ddtheta_d - lambda*de;
    
    %Get Dynamic Parameters
    M = getM(theta);
    C = getC([theta;dtheta]);
    N = get_N(theta);
    
    % Get Input
    tau = M*a + C*v + N - K*r;
    
    % Add torque to Torque List
    torque =[torque, tau];
    
    dx = zeros(8,1);
    
    dx(1:4) = x(5:8);
    dx(5:8) = inv(M)*(tau - N - C*x(5:8));
    
end

end

