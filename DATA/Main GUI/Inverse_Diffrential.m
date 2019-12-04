function [q_dot] = Inverse_Diffrential(n, temp, DH, low, high, Vx, Vy, Vz, Wx, Wy, Wz,q0)

Number_of_Links = n;
q0 = q0';
disp(DH)
for k = 1:Number_of_Links
 if temp(:,k) == 114
    L{k} = Link('d',DH(k,2), 'a', DH(k,3), 'alpha', DH(k,4));
 else
    disp(' As entered link is prismatic please enter its joint limits ')
    disp(' ')
%     limitlow(k)   = input(' Lower limit for the joint - ');
%     limitupper(k) = input(' Upper limit for the joint - ');
    L{k} = Link('theta',DH(k,1), 'a', DH(k,3), 'alpha', DH(k,4));
    L{k}.qlim = [low(k), high(k)];
    disp(' ')
 end
end
for b = 1:Number_of_Links
    X(b) = L{b};
end
n = 1:Number_of_Links;
m =[X(n)];
R = SerialLink(m)
disp(' ')
%Joint Velocities Entry
% Vx = input(' Input The End Effector Linear Velocity in X direction - ')
% Vy = input(' Input The End Effector Linear Velocity in Y direction - ')
% Vz = input(' Input The End Effector Linear Velocity in Z direction - ')
% Wx = input(' Input The End Effector Angular Velocity wrt X axis - ')
% Wy = input(' Input The End Effector Angular Velocity wrt Y axix - ')
% Wz = input(' Input The End Effector Angular Velocity wrt Z axis - ')
% q0 = input(' Input initial Joint Positions in Matrix Form, like [q1 q2 ... qn] ')
% t  = input(' Input Time at which you want Joint Variables - ')
% q0 = q0';
% q0 = zeros(1:length(n),1)
J=R.jacob0(q0);
% Time = 1:0.1:t;
Ve = [Vx Vy Vz Wx Wy Wz]';
% q= zeros(Number_of_Links,length(Time));
q(:,1) = q0'; 
q_dot = pinv(J)*Ve
 
end