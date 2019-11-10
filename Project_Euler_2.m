clc
clear all
close all

fprintf('Press 1 for zyz orientation.\nPress 2 for zyx orientation.');
orient=input(' ','s');
if xor(strcmp(orient,'1'),strcmp(orient,'2')) == 0
        fprintf('Invalid input. Press p for prismatic joint, Press r for revolute joint\n');
        %exit;
end
if orient=='1'
fprintf('Press 1 to calculate the zyz rotation matrix.\nPress 2 to calculate zyz euler angles from rotation matrix');
c=input(' ','s');
if xor(strcmp(c,'1'),strcmp(c,'2')) == 0
        fprintf('Invalid input.');
        %exit;
end
if c=='1'
   z = input('Enter the z angle');
   y = input('Enter the y angle');
   z1 = input('Enter the z1 angle');
   zyz = [cos(z)*cos(y)*cos(z1)-sin(z)*sin(z1) -cos(z)*cos(y)*cos(z1) cos(z)*sin(y); sin(z)*cos(y)*cos(z1)-cos(z)*sin(z1) -sin(z)*cos(y)*sin(z)+cos(z)*cos(z1) sin(z)*sin(z1); -sin(y)*cos(z1) sin(y)*sin(z1) cos(z)]

    
end

if c=='2'
zyz = input('Enter the zyz matrix');
z = atan2(zyz(2,3),zyz(1,3))
y = atan2((sqrt(zyz(1,3)^2+zyz(2,3)^2)),zyz(3,3))
z1 = atan2(zyz(3,2),-zyz(3,1))



end
end

if orient=='2'
fprintf('Press 1 to calculate the zyx rotation matrix.\nPress 2 to calculate zyx euler angles from rotation matrix');
c=input(' ','s');
if xor(strcmp(c,'1'),strcmp(c,'2')) == 0
        fprintf('Invalid input.');
        %exit;
end

if c=='1'
z = input('Enter the z angle');
y = input('Enter the y angle');
x = input('Enter the x angle');

zyx = [cos(z)*cos(y) cos(z)*sin(y)*sin(x)-sin(z)*cos(x) cos(z)*sin(y)*cos(x)+sin(z)*sin(x); sin(x)*cos(y) sin(z)*sin(y)*sin(x)+cos(z)*cos(x) sin(z)*sin(y)*cos(x)-cos(z)*sin(x); -sin(y) cos(y)*sin(x) cos(y)*cos(x)]

    
end

if c=='2'

zyx = input('Enter the zyx matrix')
z = atan2(zyx(2,1),zyx(1,1));
y = atan2(-zyx(3,1),sqrt(zyx(3,2)^2+zyx(3,3)^2));
x = atan2(zyx(3,2),zyx(3,3));


end
end





