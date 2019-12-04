function [] = manipulator_equation(R)
%Computes the symbolic dynamics equations given the robot

n=R.n;

%Create array of symbolic variables
syms q [1 n]

syms qd [1 n]

B=R.inertia(q)
C=R.coriolis(q,qd)
G=R.gravjac(q)

disp('The equation is of the form: B*qdd+C*qd+G=tau')

disp('The matrices are follows: (Note that for real numbers, the conjugate is the number itself)')

%Display matrices
B
C
G

end

