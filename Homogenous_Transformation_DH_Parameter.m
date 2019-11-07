%%% DENAVIT HARTENBERG Parameteres by Mahmoud KhoshGoftar%%%
clear all
close all
clc
num = input('Enter number of links');
F = ones(num,4);
B=eye(4);
C = ones(4);
clc
for i=1:num
    F(i,1)=input(['Enter a',num2str(i),':']);
    F(i,2)=input(['Enter alfa',num2str(i),':']);
    F(i,3)=input(['Enter d',num2str(i),':']);
    F(i,4)=input(['Enter theta',num2str(i),':']);
    F1=(F(i,1));
    F2=(F(i,2));
    F3=(F(i,3));
    F4=(F(i,4));
    
    C=([cos((F4)) -sin(F4)*cos(F2) sin(F4)*sin(F2) F1*cos(F4);
             sin(F4) cos(F4)*cos(F2) -cos(F4)*sin(F2) F1*sin(F4);
             0 sin(F2) cos(F2) F3;
             0 0 0 1]);
    eval(sprintf('A%d = C;',i));
    B=B*C;
    eval(sprintf('A%d',i))
end
disp(['T from link 0 to link ',num2str(i)])
disp(B)
disp('Rotation Matrix')   
R=B(1:3,1:3);
disp(R)
d=B(1:3,4);
disp('Translation Matrix')
disp(d)