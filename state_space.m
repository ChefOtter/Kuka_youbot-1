function [ A, B ] = state_space(M, C, N, x)
% Generates State space 
% Returns A and B Matrices

%dx is x_dot

A = [0 , 1;
     0 , -inv(M)*C];
 
B = [0; 
     inv(M)];


end

