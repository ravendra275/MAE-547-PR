close all
clear all
clc
%Translational matrix Specifing axis and angle of rotation%
%Number of Rotational transformation%
NRT=str2double(inputdlg('Number of Rotational Transformation ')); 
syms x y z 
theta=ones(NRT,1);
e=eye(4);
r=eye(3);
P0=zeros(1,3);
T=eye(4);
for i=1:NRT   
theta(i)=str2double(inputdlg(['Angle of rotation theta in degree ',num2str(i),' ' ]));
k(i)=(inputdlg( [num2str(i), ' Axis of Rotation (x OR y OR z) '] ));
EEPX=str2double(inputdlg('Input End Effector POsition X '));
EEPY=str2double(inputdlg('Input End Effector POsition Y '));
EEPZ=str2double(inputdlg('Input End Effector POsition Z '));

if k(i)==x
    k1=1;
    k2=0;
    k3=0;
end
if k(i)==y
    k1=0;        
    k2=1;
    k3=0;
end
if k(i)==z
    k1=0;
    k2=0;
    k3=1;
end
% k1=k(1,1);
% k2=k(2,1);
% k3=k(3,1);
st=sind(theta(i));
ct=cosd(theta(i));
vt=(1-ct);

e1=[k1^2*vt+ct        k1*k2*vt-k3*st    k1*k3*vt+k2*st EEPX;
   k1*k2*vt+k3*st    k2^2*vt+ct        k2*k3*vt-k1*st EEPY;
   k1*k3*vt-k2*st    k2*k3*vt+k1*st    k3^2*vt+ct EEPZ;
   0 0 0 1];
eval(sprintf('E%d = e1;',i));
e=e1;
eval(sprintf('E%d',i))

P1=e(1:3,4);
eval(sprintf('X%d = P1;',i));
p=P1;
eval(sprintf('X%d',i))

r1=e(1:3,1:3);
eval(sprintf('R%d = r1;',i));
r=r*r1;
eval(sprintf('R%d',i))

T=T*e1;
R=T(1:3,1:3);
P=T(1:3,4);

keyboard();
Q1=rotm2quat(R);
 eval(sprintf('QD%d = Q1;',i));
  q=Q1;
 eval(sprintf('QD%d',i))

  if i==1
   x1=[0 P(1)];
   y1=[0 P(2)];
   z1=[0 P(3)];
   plot3(x1, y1, z1,'O--') 
   else
   x1=[X1(1) T(1,4)];
   y1=[X1(2) T(2,4)];
   z1=[X1(3) T(3,4)];
   plot3(x1, y1, z1,'O--')
   end
   hold on
   grid on
 plotTransforms(P',Q1) 
end
plotTransforms(P0,[1 0 0 0])
xlabel('X-axis Red')
ylabel('Y-axis Green')
zlabel('Z-axis Blue')
disp(['Translation Matrix from 0 to ',num2str(i)] )
disp(T)
  




