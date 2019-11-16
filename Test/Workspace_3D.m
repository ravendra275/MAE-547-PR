clc
clear all
a = [0.3 0.3 0 0];alpha = zeros(1,4);
d = [0 0 0 0.3 ]; theta = zeros(1,4);
dh = [theta',d',a',alpha']
L1 = Link('d', dh(1,2), 'a', dh(1,3), 'alpha', dh(1,4));
L2 = Link('d', dh(2,2), 'a', dh(2,3), 'alpha', dh(2,4));
L3 = Link('theta', dh(3,1), 'a', dh(3,3), 'alpha', dh(3,4));
L3.qlim = [0, 0.4];
L4 = Link('d', dh(4,2), 'a', dh(4,3), 'alpha', dh(4,4));

R = SerialLink( [L1 L2 L3 L4])
q = [pi/6 -pi/2 .2 pi/3]';
W = [-4 4 -4 4 -4 4];

R.plot3d(R,q, 'workspace' , W)