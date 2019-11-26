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
q = DH(:,1);
alpha = DH(:,4);
a = DH(:,3);
d = DH(:,2);
disp(' ')
% fprintf('Enter the euler angle pose- zyz or zyx ');    
% fprintf('Press zyz for zyz end pose,\nPress zyx for zyx end pose\n');
%  
% euler=input(' zyz end pose or zyx end pose: ','s');
% if xor(strcmp(euler,'zyz'),strcmp(euler,'zyx')) == 0
%     fprintf('Invalid input. Press p for prismatic joint, Press r for revolute joint\n');
%     %exit;
% elseif (strcmp(euler,'zyz'))== 1
%     fprintf('Now we will calculate zyz transformation matrix/n');
% 
%     smallphi = input('Enter the z angle ');
%     theta = input('Enter the y angle ');
%     %psi = input('Enter the z1 angle');
%     TJ= [0 -sin(smallphi) cos(smallphi)*sin(theta);...
%         0 cos(smallphi) sin(smallphi)*sin(theta);...
%         1 0 cos(theta)]
% elseif (strcmp(euler,'zyx'))== 1
%     fprintf('Now we will calculate zyx transformation matrix/n');
%     smallphi = input('Enter the z angle ');
%     theta = input('Enter the y angle ');
%     TJ= [0 -sin(smallphi) cos(smallphi)*cos(theta);...
%         0 cos(smallphi) sin(smallphi)*cos(theta);...
%         1 0 -sin(theta)]
% end
% Ta=[eye(3) zeros(3,3);zeros(3,3) TJ]

disp(' ')
T = input(' Time of the inverse differential ');
disp(' ')
syms t;
Px  = input(' Input X position as Time Function - ');
Py  = input(' Input Y position as Time Function - ');
Pz  = input(' Input Z position as Time Function - ');
Wx = input(' Input X angle rate as Time Funtion - ');
Wy = input(' Input Y angle rate as Time Funtion - ');
Wz = input(' Input Z angle rate as Time Funtion - ');
q_in = input(' Initial Condition of joint variables in Matrix Format - ')
disp(' ')
t = 1:0.1:T;
dt = 1e-8;
pd = matlabFunction([Px; Py; Pz]);
pd_dot = (pd(t + dt) - pd(t)) / dt ;
pd = pd(t);
phid = matlabFunction([Wx;Wy;Wz]);
phid_dot = (phid(t + dt) - phid(t)) / dt ;
phid = phid(t);
q= zeros(Number_of_Links,length(t));
q(:,1) = q_in;
K = 3*eye(6);
    
for i = 1:length(t)
    Tr = R.fkine(q(:,i)');
    RPY = tr2rpy(Tr);
    xe(:,i)= [Tr(1:3,4);RPY'];
    Ja = R.jacob0(q(:,i),'rpy',RPY);
    e(:,i)=[pd(:,i); phid(:,i)] - xe(:,i);
    xd_dot=[pd_dot(:,i); phid_dot(:,i)];
    qdot = pinv(Ja)*(xd_dot+K*e(:,i));
    q(:,i+1)=q(:,i)+qdot*0.01;
end

Q=q(1:Number_of_Links,end);% Value of Joint Variables at 2.5 seconds
fprintf("Value of Joint Variables at Time");disp(T)
Q'