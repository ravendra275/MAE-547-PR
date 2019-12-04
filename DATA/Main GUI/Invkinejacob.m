function [Q] = Invkinejacob(n, temp, DH, low, high, Vx, Vy, Vz,q0)

q0 = q0';
Number_of_Links = n;
disp(DH)
for k = 1:Number_of_Links
 if temp(:,k) == 114
    L{k} = Link('d',DH(k,2), 'a', DH(k,3), 'alpha', DH(k,4));
 else
    disp(' As entered link is prismatic please enter its joint limits ')
    disp(' ')
    L{k} = Link('theta',DH(k,1), 'a', DH(k,3), 'alpha', DH(k,4));
    L{k}.qlim = [low(k), high(k)];
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
Px = Vx;
Py = Vy;
Pz = Vz;
V = [Px; Py; Pz]
Jaco = R.jacob0(q0,'trans')
Q = pinv(Jaco)*V 
plot(R,Q','workspace', [-20 20 -20 20 -20 20])
end