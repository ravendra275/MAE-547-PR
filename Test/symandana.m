clc
clear all
close all
format compact
sympref('FloatingPointOutput',true)

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

syms q [1 x];
assume(q,'real')
syms d [1 x];
assume(d,'real')
syms a [1 x];
assume(a,'real')
%syms alpha [1 x];
%assume(alpha,'real')
alpha= zeros(x,1)';

for i=1:x
    if i==1
        fprintf('Press y if parameter equals 0 else press n\n');
    end
    fprintf('Is q(%d)=',i);
    yon=input('0? ','s');
    if strcmp(yon,'y')==1
        q(i)=0
    end 
end

for i=1:x
    if i==1
        fprintf('Press y if parameter equals 0 else press n\n');
    end
    fprintf('Is d(%d)=',i);
    yon=input('0? ','s');
    if strcmp(yon,'y')==1
        d(i)=0
    end 
end
for i=1:x
    if i==1
        fprintf('Press y if parameter equals 0 else press n\n');
    end
    fprintf('Is a(%d)=',i);
    yon=input('0? ','s');
    if strcmp(yon,'y')==1
        a(i)=0
    end 
end
for i=1:x
    if i==1
        fprintf('Press y if parameter equals 0 else press n\n');
    end
    fprintf('Is alpha(%d) ',i);
    yon=input('different? ','s');
    if strcmp(yon,'y')==1
        alpha(i)= input('Enter new alpha ');
    end 
end

DH= [q; d; a; alpha]'
cosangle=zeros(1,x);
sinangle= zeros(1,x);

for i=1:x
    cosangle(i)= round(cos(alpha(i)));
    sinangle(i)= round(sin(alpha(i)));
end
digits(3);
 for i = 1:x    
    T{i} = [cos(q(i)) -sin(q(i))*round(cos(alpha(i))) sin(q(i))*round(sin(alpha(i))) a(i)*cos(q(i));
            sin(q(i)) cos(q(i))*round(cos(alpha(i))) -cos(q(i))*round(sin(alpha(i))) a(i)*sin(q(i));
            0 round(sin(alpha(i))) round(cos(alpha(i))) d(i);0 0 0 1];
 end

 T0i{1}=T{1};
 for i=1:x-1
     T0i{i+1}=T0i{i}*T{i+1};
 end
 
 p0=[0 0 0]';
 z0=[0 0 1]';
 for i=1:x
     p(1:3,i)=T0i{i}(1:3,4);
     z(1:3,i)=T0i{i}(1:3,3);
 end
 z(3,:)= simplify(round(z(3,:)))
syms Jp [3 x]
syms Jo [3 x] 
%  Jp=zeros(3,x);
%  Jo=zeros(3,x);
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
            Jp(1:3,i)= simplify(cross(z0,p(:,x)-p0));
            Jo(1:3,i)= z0;
         else
            Jp(1:3,i)= simplify(cross(z(:,i-1),p(:,x)-p(:,i-1)));
            Jo(1:3,i)= z(:,i-1);
         end
     end
 end
J=[Jp;Jo];
J=simplify(J)
[m,n]=size(J);

[m,n]=size(J);
Ja=J;
i=1;
while i<m+1
    if Ja(i,:)==zeros(1,n)
    
        Ja(i,:)=[];
        %fprintf('%d',i);
        i=i-1;
        m=m-1;
    end
    i=i+1;
end
i=1;
while i<n+1
    if Ja(:,i)==zeros(m,1)
    
     Ja(:,i)=[];
     n=n-1;
     i=i-1;
    end
    i=i+1;
end