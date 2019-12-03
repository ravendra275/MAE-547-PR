
% Assumes you already have an R, a 2 link planar robot
%Input initial q0 and qd0


R.fast=0;
xd=[1;1];

%Inputs
q0=[0 pi/6];
qt=q0;
x0=R.fkine(q0);

x0=x0(1:2,end);


%User inputs
qd0=[0 0];
qdt=qd0;

%User inputs. Default values are one each
kp=1;
kd=1;


tf=10;
dt=0.01;

n_steps=floor(tf/dt);

for i=1:n_steps
    %keyboard();
    J=R.jacob0(qt(i,:));
    J=J(1:2,1:2);
    
    J_dot=R.jacob_dot(qt(i,:),qdt(i,:));
    J_dot=J_dot(1:2); %This is already multiplied with the joint rates
    
    xt=R.fkine(qt(i,:));
    xt=xt(1:2,end);
    
    xdt=J*qdt(i,:)';
    
    y=inv(J)*(-kd*xdt+kp*(xd-xt)-J_dot);
    
    future_q=qt(i,:)+qdt(i,:)*dt;
    
    B=R.inertia(qt(i,:));
    B_dot=(R.inertia(future_q)-R.inertia(qt(i,:)))/dt;
    
    C=R.coriolis(qt(i,:),qdt(i,:));
    
    g=R.gravjac(qt(i,:));
    
    n=C*qdt(i,:)'+g'; %Refer to 8.56 in the book
    
    u=B*y+n;
    
    
    xk(:,i)=xt;
    
    
    qdd=inv(B)*(u-n); %From the robot dynamic equation
    qt(i+1,:)=qt(i,:)+qdt(i,:)*dt;
    qdt(i+1,:)=qdt(i,:)+qdd'*dt;
end

%qt represents joint angles
%qdt represents joint derivatives
    
    