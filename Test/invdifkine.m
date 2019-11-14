clc;
clear all;
syms t
double(t)
pd = [0.42*cos(t *pi/10); 0.42*sin(t*pi/10); 0.1*(1+sin(t))];
pd_dot = diff(pd);
t = 1:.1:2;
pd 
pd_dot
% q1=zeros(length(pd),length(t));
% q2=zeros(length(pd_dot),length(t));
% for t = 1:.1:2;   
%         q1(t,:) = pd;
%         q2(t,:) = pd_dot;
% end
% q1
% q2
    