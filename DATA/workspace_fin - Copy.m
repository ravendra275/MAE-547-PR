function [q] = f(q1, a, d alpha, q, DH, low, high, temp, R)
R;
temp;
high;
low;
%%%%%%%%%%%%%% Workspace for 6 DOF robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if x>=6
        t=zeros(90,x);
    for i=1:x
        if por(i)=='r'
            t(:,i)=linspace( rad2deg(q(i,1)),rad2deg(q(i,2)),90);
        elseif por(i)=='p'
            t(:,i)=linspace(q(i,1),q(i,2),90);
        end
        %T{i}=ndgrid(t{i});
    end

    for j=1:2
        for i = 1:x    
            Tr{i,j} = [cos(q(i,j)) -sin(q(i,j))*(cos(alpha(i))) sin(q(i,j))*(sin(alpha(i))) a(i)*cos(q(i,j));
                    sin(q(i,j)) cos(q(i,j))*(cos(alpha(i))) -cos(q(i,j))*(sin(alpha(i))) a(i)*sin(q(i,j));
                    0 (sin(alpha(i))) (cos(alpha(i))) d(i);0 0 0 1];
     end
    end
     T0i1{1}=Tr{1,1};
     for i=1:x-1
         T0i1{i+1}=T0i1{i}*Tr{i+1,1};
     end
    T0i2{1}=Tr{1,2};
     for i=1:x-1
         T0i2{i+1}=T0i2{i}*Tr{i+1,2};
     end

    T1=[eye(3) T0i1{2}(1:3,4);0 0 0 1];
    t2=[eye(3) T0i2{2}(1:3,4);0 0 0 1];
    w=zeros(10000,x);

    % m=zeros(1,6)
    % m1=0
    % while m1<x
    %     r=randperm(6,1)
    %     if m(r)~=1
    %         m(r)=1;
    %         m1=m1+1;
    %     end
    % end
     m=[1 0 0 1 1 1];
    %q12=R.ikunc(T1,T0i2{2}(1:4,4)')
    
    if x>=6
        w=R.jtraj(T1, t2, 10000);
    else
        w=R.jtraj(T1,t2,10000,R.ikine(T1,T0i2{2}(1:4,4)',m','pinv'));
    end

    q_f1=R.fkine(w);

    x_traj = zeros(1,10000);
    y_traj = zeros(1,10000);
    z_traj = zeros(1,10000);

    for i=1:10000
    x_traj(1,i) = q_f1(1,4,i);
    y_traj(1,i) = q_f1(2,4,i);
    z_traj(1,i) = q_f1(3,4,i);
    end

    figure(1)
    workspx=reshape(x_traj(1,:),[2000,5]);
    workspy=reshape(y_traj(1,:),[2000,5]);
    workspz=reshape(z_traj(1,:),[2000,5]);
    surf(workspx(:,:),workspy(:,:),workspz(:,:));

    figure(2)
    hold on
    for i=1:10000
    %R.plot([w(i,:)],'fps',200);
    [x,y,z] = sphere(16);
    plot3(x_traj(1,i),y_traj(1,i),z_traj(1,i),'.');
    hold on;
    end
    hold on;

    vstup=0;
    while (vstup ~= 1)
    R.plot(w,'delay',1e-20)
    end
    hold off;
end

 
%%%% Workspace for underactuated robots %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if x<6
    t=zeros(1000,x);
    for i=1:x
        if por(i)=='r'
            t(:,i)=linspace( rad2deg(q(i,1)),rad2deg(q(i,2)),1000);
        elseif por(i)=='p'
            t(:,i)=linspace(q(i,1),q(i,2),1000);
        end
    end
    
    figure(1);
    N=30;
    J = R.fkine(t);  
    for i= 0:N
        for j= 1:N+1
            TR= J(:,4,i*(30)+j)
            workspace(i+1,j,:) = TR;
        end
    end
    surf(workspace(:,:,1), workspace(:,:,2), workspace(:,:,3));
    
    space=[0:.01:10];
    [Q QD QDD]=jtraj(q(:,1),q(:,2),space);
    % J=fkine(R, Q);
     
    figure(2);
    for x=1:1:1000
        plot3(J(1,4,x),J(2,4,x),J(3,4,x),'b.')
        hold on;
       % R.plot([t(x,:)],'workspace',[-1 1 -1 1 -1 1]);
    end
    hold on; 
    vstup=0;
    while (vstup ~= 1)
    R.plot(Q,'delay',1e-20)
    end
    hold off;
end
end