%Rotational matrix Specifing axis and angle of rotation for %
%Number of Rotational transformation%
NRT=input('Number of Rotational Transformation '); 
syms x y z 
theta=ones(NRT,1);
R=eye(3);
for i=1:NRT
theta(i)=input(['Angle of rotation theta ',num2str(i),' ' ]);
k(i)=input( [num2str(i), ' Axis of Rotation (x OR y OR z) '] );
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
st=sin(theta(i));
ct=cos(theta(i));
vt=(1-ct);
R1=[k1^2*vt+ct        k1*k2*vt-k3*st    k1*k3*vt+k2*st;
   k1*k2*vt+k3*st    k2^2*vt+ct        k2*k3*vt-k1*st;
   k1*k3*vt-k2*st    k2*k3*vt+k1*st    k3^2*vt+ct     ];
R=R*R1;
end
disp(['Roatation Matrix from 1 to ',num2str(i)] )
disp(R)
%translation - End Effector Position%
EEPX=input('Input End Effector POsition X ');
EEPY=input('Input End Effector POsition Y ');
EEPZ=input('Input End Effector POsition Z ');
P=[EEPX;EEPY;EEPZ];
disp(['Translation Matrix from 1 to ',num2str(i)] )
disp(P)
%Transformational Matrix%
T=[R,P;0 0 0 1];
disp(['Transformational Matrix from 1 to ',num2str(i)] )
disp(T)

