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
        theta(i,1) = input(' Angle theta in radians - ');
        disp(' ')
    else
        disp(' For Defined Prismatic Joint ')
        a(i,1)     = input(' Link Length - ');
        d(i,1)     = input(' Distance d - ');
        alpha(i,1) = input(' Angle alpha in radians - ');
        theta(i,1) = input(' Angle theta in radians - ');
        disp(' ')
    end
end
disp(' ')
disp(' Below are the inputed DH Parameters ')
disp(' in order of theta d a alpha ')
DH = [theta d a alpha]
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
phi = input(' Input total angle phi till the end effector - ');
Px  = input(' Input Position in wrt to X axis - ');
Py  = input(' Input Position in wrt to Y axis - ');
Pz  = input(' Input Position in wrt to Z axis - ');
T = [ cos(phi) -sin(phi) 0 Px;
      sin(phi)  cos(phi) 0 Py;
      0 0 1 Pz;
      0 0 0 1];

disp(' ');
disp(' The Joint parameters from given end effector pose are - ')
INVS = R.ikunc(T) % using the unconstrained inverse kinematic function finding the inverse for the robot
