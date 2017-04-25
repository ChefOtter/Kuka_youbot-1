function [] = inverse_dynamics(trajectory, tf )
%Passivity Based Control for Ideal Dynamics
% tau = M*a + C*v + N - Kr
% r = de + lambda*e
% v = dq - lambda*e
% a = ddq = lambda*de

%theta = [q1, q2, q3, q4, dq1, dq2, dq3, dq4]
x0e=rand(10,1);
x0e = [0.0033;
    0.3186;
    0.8892;
    0.0961;
    0.0190;
    0.0369;
    0.5347;
    0.6767;
    0.2529;
    0.0856];
x0e(1) = 0;
x0e(5) = 0;
x0e(6) = 0;
x0e(10) = 0;
torque = [];

options = odeset('RelTol',1e-4,'AbsTol',[1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4]);
[T,X] = ode45(@(t,x)planarArmODEUncertain(t,x),[0 tf],x0e,options);

figure('Name','theta1');
plot(T, X(:,2),'r-');
title('Under Control')
hold on

plot(T, trajectory(2,1)+trajectory(2,2)*T+ trajectory(2,3)*T.^2+trajectory(2,4)*T.^3, 'b-');
title('Theta_2 under Robust Control');

function [dx ] = planarArmODEUncertain(t,x)
    %disp('In ODE...')
    K = 10*eye(5);
    Kp = 10*eye(5);
    Kd = 2*eye(5);
    lambda = 10*eye(5);
    vec_t = [1; t; t^2; t^3]; % cubic polynomials
    theta_d= [trajectory(1,:)*vec_t; trajectory(2,:)*vec_t; trajectory(3,:)*vec_t; trajectory(4,:)*vec_t; trajectory(5,:)*vec_t]; % chnaged transpose
    
    
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
    
    % Joint 5 Velcoity and Acceleration
    a5_vel = [trajectory(5,2), 2*trajectory(5,3), 3*trajectory(5,4), 0];
    a5_acc = [2*trajectory(5,3), 6*trajectory(5,4),0,0 ];
    
    % Desired Velocities
    dtheta_d =[a1_vel*vec_t; a2_vel* vec_t; a3_vel* vec_t; a4_vel* vec_t; a5_vel*vec_t];
    
    %Desired Accelerations
    ddtheta_d =[a1_acc*vec_t; a2_acc* vec_t; a3_acc* vec_t; a4_acc* vec_t; a5_acc*vec_t];
    
    %Actual Velocities and Acceleration
    theta= x(1:5,1);
    dtheta= x(6:10,1);
    
    
    %Get Errors
    e = theta - theta_d;
    de = dtheta - dtheta_d;
    error = [e;de];
    
    %Get Passivity Parameters
    %r = de + lambda*e;
    %v = dtheta_d - lambda*e;
    %a = ddtheta_d - lambda*de;
    
    
    
    %Get Dynamic Parameters
    M = getM(theta);
    C = getC(theta,dtheta);
    N = get_N(theta);
    
    % Get Input
    %tau = M*a + C*v + N - K*r;
    tau = M*(-Kp*e -Kd*de) + C*dtheta + N;
    
    % Add torque to Torque List
    torque =[torque, tau];
    
    dx = zeros(8,1);
    
    dx(1:5) = x(6:10);
    dx(6:10) = inv(M)*(tau - N - C*x(6:10));
    
end

end

