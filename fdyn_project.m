function [R]=fdyn_project(R)
%Script for simulating the forward dynamics of a robot given the
%generalized torques as a function of time.

%SerialLink Object already available from the robot definition

disp('We will require the mass, location of the CG, and the inertia matrix about CG for each link:')

n_joints=R.n;


for i=1:n_joints
    fprintf('For Link %d, ',i)
    m=input('Mass of Link= ')
    r=input('Position of CG wrt link frame (a row vector)= ')
    I=input('Inertia of Link wrt CG (input 3x3 matrix)= ')
    
    Jm=input('Inertia of the motor driving this link= ')
    G=input('Gear Ratio')
    
    R1.links(i).m=m;
    R1.links(i).r=r;
    R1.links(i).I=I;

    R1.links(i).Jm=Jm;
    R1.links(i).G=G;
    
end

syms t

flag=false;
while(~flag)
    t_or_q=input('Do you want to try open loop (define torque wrt t) or joint feedback(wrt q)')
    if(t_or_q=='t' || t_or_q=='q')
        flag=true
    else
        disp('Please input t or q: ')
    end
end


if(t_or_q=='t')
    tau=input('Input torque as a function of time eg sin(t): ')
else
    tau=input('Input torque as a function of joint angles eg q: ')
end

tau=matlabFunction(t,q,qd,a);

%Input initial conditions
q0=input('Initial joint position (in meters or radians):')
qd0=input('Initial joint derivatives (in m/s or rad/s): ')

[T,q,qd]=R.fdyn(@tau,q0,qd0)

%Write plotting routine now