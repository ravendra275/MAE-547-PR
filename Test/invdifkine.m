clc;
clear all;
syms t

pd = [0.42*cos(t*pi/10); 0.42*sin(t*pi/10); 0.1*(1+sin(t))];
phid = t*pi/10+7*pi/12;
pd_dot = diff(pd);
phid_dot = diff(phid);
xd_dot = [pd_dot;phid_dot];
K = 1*eye(4,4)

