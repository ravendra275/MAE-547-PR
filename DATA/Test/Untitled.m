K = eye(4)

for i = 1:length(t)
    T = R.fkine(q(:,i)');
    xe(:,i)= T(:,4);
    J = R.jacob0(q(:,i));
    Ja = inv(Ta)*J;
    e(:,i)=[pd(:,i); phid(i)] - xe(:,i);
    xd_dot=[pd_dot(:,i); phid_dot(:,i)];
    qdot = Ja'.*(K*e(:,i));
    q(:,i+1)=q(:,i)+qdot*0.01;
end
