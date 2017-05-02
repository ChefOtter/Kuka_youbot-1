# Kuka_youbot
For expressing the dynamic model of kuka youbot robotic arm in 

M*q_dd+C*q_d+N=T

M: Inertia Matrix

C: Coriolis Matrix

N: Gravity Matrix

T: Torque

forwardkin.m is used for finding end effector position from joint angle 

getC is to get Coriolis matrix given q,q_d

getM is to get Inetia matrix given q

getN is to get Gravity matrix given q



Main File
------------
To run the simulation, run main.m 

40 and 41 lines call Inverse Dynamics and Passivity control (you can comment or uncomment them)
