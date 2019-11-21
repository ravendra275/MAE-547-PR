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
q=zeros(x,2);
for i=1:x
    fprintf('Enter q(%d) limits:',i);
    for j=1:2
        fprintf('From ');
        q(i,j)=input('');
        fprintf('To \n');
    end
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
 
 
L=zeros(x,3);
for i=1:x
    L(i)=Link(d(i),a(i),alpha(i));
end

R=SerialLink(L,'qlim',q);