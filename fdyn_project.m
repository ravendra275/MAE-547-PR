function [R]=fdyn_project(R)

%Script for simulating the forward dynamics of a robot given the
%generalized torques as a function of time.

%SerialLink Object already available from the robot definition

disp('We will require the mass, location of the CG, and the inertia matrix about CG for each link:')
syms t q qd a y n
n_joints=R.n;
%{

g=input('Input gravity vector in the base frame(as a constant row vector): ')

R1.gravity=g';

for i=1:n_joints
    fprintf('For Link %d, ',i)
    m=input('Mass of Link= ')
    r=input('Position of CG wrt link frame (a row vector)= ')
    I=input('Inertia of Link wrt CG (input 3x3 matrix)= ')
    
    Jm=input('Inertia of the motor driving this link= ')
    G=input('Gear Ratio:- ')
    
    R.links(i).m=m;
    R.links(i).r=r;
    R.links(i).I=I;

    R.links(i).Jm=Jm;
    R.links(i).G=G;
    
end

keyboard();




%}
flag=false;

while(~flag)
    t_or_q=input('Do you want to try open loop (define torque wrt t) or joint feedback(wrt q)')
    if(t_or_q=='t' || t_or_q=='q')
        flag=true
    else
        disp('Please input t or q: ')
    end
end


disp('You will be asked to input the torque function. Please define it as a row vector')

if(t_or_q=='t')
    constant_or_not=input('Are the torques ALL constant (y or n)?: ')
    if(constant_or_not=='n')
        temp=input('Input torque as a row vector function of time eg [sin(t) cos(t)]: ')
        asd=matlabFunction(temp)
        torq_fun=@(R,t,q,qd) asd(t)
    else
        temp=input('Input torque as a row vector of constant values: ')
        torq_fun=@(R,t,q,qd) temp(t)
    end
else
    constant_or_not=input('Are the torques ALL constant (y or n)?: ')
    if(constant_or_not=='n')
        temp=input('Input torque as a function of joint angles eg q: ')
        asd=matlabFunction(temp)
        torq_fun=@(R,t,q,qd) asd(q)
    else
        temp=input('Input the torque as a row of constants: ')
        torq_fun=@(R,t,q,qd) temp
    end
end


syms q [1 n_joints]
syms qd [1 n_joints]

B=

%Input initial conditions
q0=input('Initial joint position (in meters or radians):')
qd0=input('Initial joint derivatives (in m/s or rad/s): ')

tsim=input('How long (in seconds) do you want to run the simulation for?:- ')


%keyboard();
[T,angles,angle_derivatives]=R.fdyn(tsim,torq_fun,q0,qd0)

%Write plotting routine now

for i=1:n_joints
figure(1)
hold on
plot(T,180/pi*angles(:,i))
title('Joint angles')

end

for i=1:n_joints
figure(2)
hold on
plot(T,180/pi*angle_derivatives(:,i))
end

end
