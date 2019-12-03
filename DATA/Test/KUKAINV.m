clear all
clc
alpha = [pi/2, -pi/2, -pi/2, pi/2, pi/2, -pi/2, 0];  a = zeros(1,7);
d = [0.3105, 0, 0.4 , 0, 0.39, 0, 0.083];
theta = zeros(1,7);
dh = [theta' d' a' alpha'];

for i = 1:7
    L2{i} = Link('d', dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4));
end
R = SerialLink([L2{1} L2{2} L2{3} L2{4} L2{5} L2{6} L2{7}])


qd =[.4 .5 .6 .7 .8 .9 1]

% [q1, err] = R2.ikcon(T)
%j = R2.jacob0(q1,  'rpy')
Td = R.fkine([qd]) 
W=[-1 1 -2 2 -2 2];
%plot(R,qd,'workspace', W)
%teach(R)
m=[0 0 0 1 1 1]';
q0=[.1 .2 .3 .4 .5 .6 .7];  
qcomp = R.ikine(Td, q0, m, 'pinv') 
%qcomp = R.ikunc(Td)
Tcomp = R.fkine(qcomp)