clc
clear all
close all

x= input('Enter number of links: ');
for i=1:x
    if i==1
        fprintf('Press p for prismatic joint,\nPress r for revolute joint\n');
    end
    fprintf('Is Link %d',i)
    por(i)=input(' prismatic or revolute: ','s');
    if xor(strcmp(por(i),'r'),strcmp(por(i),'p')) == 0
        fprintf('Invalid input. Press p for prismatic joint, Press r for revolute joint\n');
        %exit;
    end
end

a=zeros(1,x);
for i=1:x
    fprintf('Enter a(%d):',i);
    a(i)=input('');
end
d=zeros(1,x);
for i=1:x
    fprintf('Enter d(%d):',i);
    d(i)=input('');
end
q=zeros(1,x);
for i=1:x
    fprintf('Enter q(%d):',i);
    q(i)=input('');
end
alpha=zeros(1,x);
for i=1:x
    fprintf('Enter alpha(%d):',i);
    alpha(i)=input('');
end
lno=[1:x];
DH= [lno; q; d; a; alpha]'
 DH=array2table(DH,...
     'VariableNames',{'Link no.','q','d','a','alpha'})

 for i = 1:x    
    T{i} = [cos(q(i)) -sin(q(i))*cos(alpha(i)) sin(q(i))*sin(alpha(i)) a(i)*cos(q(i));
            sin(q(i)) cos(q(i))*cos(alpha(i)) -cos(q(i))*sin(alpha(i)) a(i)*sin(q(i));
            0 sin(alpha(i)) cos(alpha(i)) d(i);        0 0 0 1];
 end

 T0i{1}=T{1};
 for i=1:x-1
     T0i{i+1}=T0i{i}*T{i+1};
 end
 
 p0=[0 0 0]';
 z0=[0 0 1]'
 for i=1:x
     p(1:3,i)=T0i{i}(1:3,4);
     z(1:3,i)=T0i{i}(1:3,3);
 end
 
 for i=1:x
     if por(i)=='p'
         if i==1
            Jp(1:3,i)= z0;
            Jo(1:3,i)= [0 0 0]';
         else
            Jp(1:3,i)= z(:,i-1);
            Jo(1:3,i)= [0 0 0]';
         end
         
     elseif por(i)=='r'
         if i==1
            Jp(1:3,i)= cross(z0,p(:,x)-p0);
            Jo(1:3,i)= z0;
         else
            Jp(1:3,i)= cross(z(:,i-1),p(:,x)-p(:,i-1));
            Jo(1:3,i)= z(:,i-1);
         end
     end
 end
 J=[Jp;Jo]
