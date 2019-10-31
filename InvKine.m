addpath(genpath('rvctools'));

% Lets define the DH parmetes
Number_of_Links = input(' Enter the Number of Links in the Robot - ');
disp(' ');
Type_of_Links   = input(' Enter Type of Links Revolute or Prismatic, All would be same!! enter 1 or 2 (case sensitive) - ');
disp(' ');
disp(' Now enter the prompted values in ascending order from link i to n')
for i = 1:Number_of_Links
    a(i,1)     = input(' Link Length - ');
    d(i,1)     = input(' Distance d - ');
    theta(i,1) = input(' Angle theta in radians - ');
    alpha(i,1) = input(' Angle alpha in radians - ');
end
disp(' ');
disp(' Entered DH Parametes are - ');
disp(' ');
disp(' a  d  theta  alpha');
dh = [theta d a alpha]
% Making links using RVC Tools
if Type_of_Links==1
for i = 1:Number_of_Links
    L1{i} = Link('d',dh(i,2), 'a', dh(i,3), 'alpha', dh(i,4));
end
R = SerialLink([L1{1} L1{2} L1{3}])
phi = input(' Input total angle phi till the end effector - ');
Px  = input(' Input Position in wrt to X axis - ');
Py  = input(' Input Position in wrt to Y axis - ');
Pz  = input(' Input Position in wrt to Z axis - ');
T = [ cos(phi) -sin(phi) 0 Px;
      sin(phi)  cos(phi) 0 Py;
      0 0 1 Pz;
      0 0 0 1];
end

INVS = R.ikunc(T)
