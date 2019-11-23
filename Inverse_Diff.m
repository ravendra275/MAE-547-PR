clc
clear all
% Inputing the number of links
Number_of_Links = input(' Enter the Number of Links in the Robot - ');
disp(' ');
for i=1:Number_of_Links
    if i==1
        fprintf('Press p for prismatic joint,\nPress r for revolute joint\n');
    end
    fprintf('Is Link %d',i)
    por(i)=input(' prismatic or revolute: ','s');
    if xor(strcmp(por(i),'r'),strcmp(por(i),'p')) == 0
        fprintf('Invalid input. Press p for prismatic joint, Press r for revolute joint\n');
        %exit;
    end
end
XX = input(' enter 1 for DH or 2 for Link Defination - ');
disp(' ')
disp(' Now enter the prompted values in ascending order from link i to n, these values depend on the link type type defined above ')
disp(' ')
if XX == 1 
for i = 1:Number_of_Links
    if por(i)=='r'
        disp(' For Defined Revolute Joint ')
        a(i,1)     = input(' Link Length - ');
        d(i,1)     = input(' Distance d - ');
        alpha(i,1) = input(' Angle alpha in radians - ');
        disp(' ')
    else
        disp(' For Defined Prismatic Joint ')
        a(i,1)     = input(' Link Length - ');
        alpha(i,1) = input(' Angle alpha in radians - ');
        theta(i,1) = input(' Angle theta in radians - ');
        disp(' ')
    end
    if por(i)=='r'
        DH(i,:) = [0 d(i,1) a(i,1) alpha(i,1)];
    else
        DH(i,:) = [theta(i,1) 0 a(i,1) alpha(i,1)];
    end
end
disp(' ')
disp(' Below are the inputed DH Parameters ')
disp(' in order of theta d a alpha ')
else
for i = 1:Number_of_Links
    if por(i)=='r'
        disp(' For Defined Revolute Joint ')
        linkl(i,1)     = input(' Length of link - ');
        zdist(i,1)     = input(' Distance between joints in Z axis direction - ');
        zangle(i,1) = input(' Angle between the z axis of consecutive links in radians - ');
        disp(' ')
    else
        disp(' For Defined Prismatic Joint ')
        linkl(i,1)  = input(' Length of link - ');
        zangle(i,1) = input(' Angle between the z axis of the consecutive links in radians - ');
        xangle(i,1) = input(' Angle between the x axis of the consecutive links in radians - ');
        disp(' ')
    end
    if por(i)=='r'
        DH(i,:) = [0 zdist(i,1) linkl(i,1) zangle(i,1)];
    else
        DH(i,:) = [xangle(i,1) 0 linkl(i,1) zangle(i,1)];
    end
end
end
disp(DH)
for k = 1:Number_of_Links
 if por(k)=='r'
    L{k} = Link('d',DH(k,2), 'a', DH(k,3), 'alpha', DH(k,4));
 else
    disp(' As entered link is prismatic please enter its joint limits ')
    disp(' ')
    limitlow(k)   = input(' Lower limit for the joint - ');
    limitupper(k) = input(' Upper limit for the joint - ');
    L{k} = Link('theta',DH(k,1), 'a', DH(k,3), 'alpha', DH(k,4));
    L{k}.qlim = [limitlow(k), limitupper(k)];
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
Vx = input(' Input The End Effector Linear Velocity in X direction - ')
Vy = input(' Input The End Effector Linear Velocity in Y direction - ')
Vz = input(' Input The End Effector Linear Velocity in Z direction - ')
Wx = input(' Input The End Effector Angular Velocity wrt X axis - ')
Wy = input(' Input The End Effector Angular Velocity wrt Y axix - ')
Wz = input(' Input The End Effector Angular Velocity wrt Z axis - ')
q0 = input(' Input initial Joint Positions in Matrix Form, like [q1 q2 ... qn] ')
% t  = input(' Input Time at which you want Joint Variables - ')
q0 = q0';

J=R.jacob0(q0);
% Time = 1:0.1:t;
Ve = [Vx Vy Vz Wx Wy Wz]';
% q= zeros(Number_of_Links,length(Time));
q(:,1) = q0'; 
q_dot = pinv(J)*Ve
 
% for i = 1:length(Time)
% % q = q0 + q_dot*t;
% q(:,i+1)=q(:,i)+q_dot*0.01;
% end
% 
% 
% disp(' ')
% disp([' The Joint Variables at the ',num2str(t),'th second are - '])
% disp(q(:,end))