addpath(genpath('rvctools'));

% Lets define the DH parmetes
Number_of_Links = input('Enter the Number of Links in the Robot - ');
disp(' Now enter the prompted values in ascending order from link i to n')
for i = 1:Number_of_Links
    a(i,1)     = input('Link Length - ');
    d(i,1)     = input('Distance d - ');
    theta(i,1) = input('Angle theta in radians - ');
    alpha(i,1) = input('Angle alpha in radians - ');
end

disp(' Entered DH Parametes are - ');

disp(' a  d  theta  alpha');
[a,d,theta,alpha]