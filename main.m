clc
clear all
close all

q = rand(5,1);
qd = rand(5,1);

M = getM(q);
C = getC(q,qd);
N = get_N(q);

[A,B] = state_space(M,C,N,[q;qd]);
