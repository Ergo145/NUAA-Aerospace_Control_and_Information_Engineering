close all
clear all
clc

N = 7000;
P=3;Q=2;sigmau=1;AR=-[-0.579 -0.442 0.769];MA=[-0.494 0.297];
% P=4;Q=3;sigmau=1;AR=-[-0.660 0.385 -0.646 0.739];MA=[-0.531 0.219 -0.416];
% P=6;Q=4;sigmau=1;AR=[1.2798 -0.7805 0.1635 -0.7566 1.0621 -0.7821];MA=[-0.2997 0.4147 -0.2794 0.4973];

u=normrnd(0,sigmau,N,1);
u=[zeros(P,1);u];
X=zeros(P,1);
MAT=[1,MA];
MAF=fliplr(MAT);
ARF=fliplr(AR);%翻转向量使之与数据的增长方向一致
for i=(P+1):(N+P),
    U=u((i-Q):i);%更新U
    x(i)=ARF*X+MAF*U;
    X=[X(2:P);x(i)];%更新X
end
DSx=(x((P+1):(N+P)))';

DSx = DSx-mean(DSx);

S = 3*P;

for i=0:S
    sum(i+1)=0;
    for j=1:(N-i),
        sum(i+1)=sum(i+1)+DSx(j)*DSx(j+i);
    end
    Ryy(i+1)=sum(i+1)/N;
end
Ryy=Ryy';

for i=1:(S-Q)
    R1(i,1) = Ryy(1+abs(Q+i));
    for j=1:P
        Ry(i,j)=Ryy(1+abs(i-j+Q));
    end
end

ARpara = inv(Ry'*Ry)*Ry'*R1;

for i=1:N
    ResDS(i)=DSx(i);
    for j=1:P
        if(i>j)
            ResDS(i)=ResDS(i)-ARpara(j)*DSx(i-j);
        end
    end
end

for i=0:P+1
    sum1(i+1)=0;
    for j=1:(N-i),
        sum1(i+1)=sum1(i+1)+ResDS(j)*ResDS(j+i);
    end
    Rfy(i+1)=sum1(i+1)/N;
end
Rfy=Rfy';
ARpara1=[1,ARpara'];
for tao=1:(Q+1),%0到Q阶MA（Q）序列的自相关计算
    Rfx(tao)=Rfy(tao);
%     for k=1:(P+1-tao),
%         Rfx(tao)=Rfx(tao);
%     end
end

[a,b]=size(Rfx);
if(a==1),
    Rfx=Rfx';
end
R0=Rfx(1);
if(Rfx(1)~=1),
    Rfx=Rfx/R0;
end
LoopN=100;
R=Rfx';
[a,b]=size(R);
m=min(a,b);
n=size(R,2)/m-1;
R(:,m*(n+2):m*(LoopN+10))=zeros;
Rre(1:m,1:m)=R(:,1:m);
for t=1:LoopN;
    Rre(t*m+1:(t+1)*m,1:m)=R(:,t*m+1:(t+1)*m);
    for i=t-1:-1:0;
        sum=zeros(m,m);
        for s=i+1:min(n,t)
            sum=sum+Rre(t*m+1:(t+1)*m,(t-s)*m+1:(t-s+1)*m)/(Rre((t-s)*m+1:(t-s+1)*m,(t-s)*m+1:(t-s+1)*m))*Rre((t-i)*m+1:(t-i+1)*m,(t-s)*m+1:(t-s+1)*m);
        end
        Rre(t*m+1:(t+1)*m,(t-i)*m+1:(t-i+1)*m)=R(:,i*m+1:(i+1)*m)-sum;
    end
end
for i=1:LoopN
    qe(:,(i-1)*m+1:m*i)=Rre((i-1)*m+1:m*i,(i-1)*m+1:m*i);
end
for j=1:n
    for t=1+j:LoopN
        d((j-1)*m+1:j*m,(t-1)*m+1:t*m)=Rre((t-1)*m+1:t*m,(t-j-1)*m+1:(t-j)*m)/(qe(:,(t-1)*m+1:m*t));
    end
end
MApara=d(1:Q,LoopN)';%GW迭代计算的MA向量参数
BMA=[1,MApara];
PrnV=R0/norm(BMA,2)^2;

t=1:N;
figure(1)
plot(t,DSx);xlabel('Samples'),ylabel('\it\rm{x}')
figure(2)
plot(t,ResDS);xlabel('Samples'),ylabel('\it\rm{x}')