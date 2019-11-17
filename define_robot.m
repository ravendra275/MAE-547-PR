% Inputing the number of links
%{
Number_of_Joints = input(' Enter the Number of Links in the Robot - ');
disp(' ');
i=1;
%The for loop has been replaced by a while loop
while(i<=Number_of_Joints)
    if i==1
        fprintf('Press p for prismatic joint,\nPress r for revolute joint\n');
    end
    fprintf('Is Link %d',i)
    por(i)=input(' prismatic or revolute: ','s'); %Array storing joint types
    if xor(strcmp(por(i),'r'),strcmp(por(i),'p')) == 0
        i=i-1;
        fprintf('Invalid input. Press p for prismatic joint, Press r for revolute joint\n');
        %exit;
    end
    i=i+1;
end
XX = input(' enter 1 for DH parameters or 2 if you are unsure - ');
disp(' ')
disp(' Now enter the prompted values in ascending order from link i to n, these values depend on the link type type defined above ')
disp(' ')
if XX == 1 
for i = 1:Number_of_Joints
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
disp(DH)
else
for i = 1:Number_of_Joints
    if por(i)=='r'
        disp(' For Defined Revolute Joint ')
        linkl(i,1)     = input(' Length of link - ');
        zdist(i,1)     = input(' Distance between joints along the previous joint axis - ');%Include this in the manual
        zangle(i,1) = pi/180*input(' Angle between the z axis of consecutive joints in degrees - ');
        disp(' ')
    else
        disp(' For Defined Prismatic Joint ')
        linkl(i,1)  = input(' Length of link - ');
        zangle(i,1) = pi/180*input(' Angle between the z axis of the consecutive links in degrees - ');
        xangle(i,1) = pi/180*input(' Angle between the x axis of the consecutive links in degrees - ');
        disp(' ')
    end
    if por(i)=='r'
        DH(i,:) = [0 zdist(i,1) linkl(i,1) zangle(i,1)];
    else
        DH(i,:) = [xangle(i,1) 0 linkl(i,1) zangle(i,1)];
    end
end
end
    
%% Making links using RVC Tools
% Link_array=[]
for k = 1:Number_of_Joints
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
for b = 1:Number_of_Joints
    X(b) = L{b};
end
n = 1:Number_of_Joints;
m =[X(n)];
%The following is the robot object
R = SerialLink(m);
%}
