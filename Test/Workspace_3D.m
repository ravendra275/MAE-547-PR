clc
clear all
a = [2 2 0 0];alpha = zeros(1,4);
d = [0 0 0 0.5 ]; theta = zeros(1,4);
dh = [theta',d',a',alpha']
L1 = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4));
L2 = Link('d', dh(2,2), 'a', dh(2,3), 'alpha', dh(2,4));

L3 = Link('theta', dh(3,1), 'a', dh(3,3), 'alpha', dh(3,4));
L3.qlim = [0, 1];
L4 = Link('d', dh(4,2), 'a', dh(4,3), 'alpha', dh(4,4));
q_lim = [-pi        pi;
         -pi/4    pi/4;
         -0.1  1;
         -pi              pi;];

R = SerialLink([L1 L2 L3 L4],'qlim',q_lim)
t = 1:0.1:4;
q = [pi/2*t; -pi/2*t; 0.1*t; pi/2*t]';
W = [-5 5 -5 5 -5 5];
% figure(1)
% plot(R ,q ,'workspace',W)
% R.animate(q)
t1 = linspace(0,360,90)*pi/180;
t2 = linspace(-120,120,90)*pi/180;
t3 = linspace(-90,90,90)*pi/180;
hold on

d3 = linspace(0,0.5,40);
d4 = linspace(0,1,40);
[T1,T2,T3]=ndgrid(t1,t2,t3);
xM = (dh(1,3))*cos(T1)+(dh(2,3))*cos(T1+T2)+2*cos(T1+T2+T3);
yM = (dh(1,3))*sin(T1)+(dh(2,3))*sin(T1+T2)+2*sin(T1+T2+T3);
zM = zeros(1,length(xM(:)));

%  figure(2)
plot3(xM(:),yM(:),zM(:),'--');
%plot(R ,q ,'workspace',W)
% R.animate(q)
% teach(R)
