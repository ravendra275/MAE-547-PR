function [Q] = Invkinejacob(n, temp, DH, low, high, ff,ET, T, q0)

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
t = 1:0.1:T;

dt = 1e-8;
for j = 1:length(t)
    for i = 1:0.1:T
        P1(:,j) = (ff(i + dt) - ff(i))/dt;
        P1_dot(:,j) = ff(i);
    end
end

q= zeros(Number_of_Links,length(t));
q(:,1) = q0;
K = 3*eye(6);
    
for i = 1:length(t)
    if ET == 'RPY'
    Tr = R.fkine(q(:,i)');
    RPY = tr2rpy(Tr);
    if rcond(eul2jac(RPY)) == 0
        xe(:,i)= [Tr(1:3,4)];
        Ja = R.jacobn(q(:,i), 'trans')
        e(:,i)= P1(1:3,i) - xe(:,i);
        xd_dot=P1_dot(1:3,i);
        qdot = pinv(Ja)*(xd_dot+K(1:3,1:3)*e(1:3,i));
        q(:,i+1)=q(:,i)+qdot*0.01;
    else
        xe(:,i)= [Tr(1:3,4);RPY'];
        Ja = R.jacob0(q(:,i),'rpy',RPY);
        e(:,i)= P1(:,i) - xe(:,i);
        xd_dot=P1_dot(:,i);
        qdot = pinv(Ja)*(xd_dot+K*e(:,i));
        q(:,i+1)=q(:,i)+qdot*0.01;
    end
    else
    Tr = R.fkine(q(:,i)');
    EUL = tr2eul(Tr);
        if rcond(eul2jac(EUL)) == 0
        xe(:,i)= [Tr(1:3,4)];
        Ja = R.jacobn(q(:,i), 'trans')
        e(:,i)= P1(1:3,i) - xe(:,i);
        xd_dot=P1_dot(1:3,i);
        qdot = pinv(Ja)*(xd_dot+K(1:3,1:3)*e(1:3,i));
        q(:,i+1)=q(:,i)+qdot*0.01;
        else
        xe(:,i)= [Tr(1:3,4);EUL'];
        Ja = R.jacob0(q(:,i),'eul',EUL);
        e(:,i)= P1(:,i) - xe(:,i);
        xd_dot=P1_dot(:,i);
        qdot = pinv(Ja)*(xd_dot+K*e(:,i));
        q(:,i+1)=q(:,i)+qdot*0.01;
        end
    end
%     xe(:,i)= [Tr(1:3,4);EUL'];
%     Ja = R.jacob0(q(:,i),'eul',EUL);
%     end
%     e(:,i)= P1(:,i) - xe(:,i);
%     xd_dot=P1_dot(:,i);
%     qdot = pinv(Ja)*(xd_dot+K*e(:,i));
%     q(:,i+1)=q(:,i)+qdot*0.01;
end

Q=q(1:Number_of_Links,end);% Value of Joint Variables at 2.5 seconds
fprintf("Value of Joint Variables at Time");disp(T)
Q'
end