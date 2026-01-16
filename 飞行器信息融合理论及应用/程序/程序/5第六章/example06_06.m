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
MA=[1,MA];
MA=fliplr(MA);
AR=fliplr(AR);%翻转向量使之与数据的增长方向一致
for i=(P+1):(N+P),
    U=u((i-Q):i);%更新U
    x(i)=AR*X+MA*U;
    X=[X(2:P);x(i)];%更新X
end
DSx=(x((P+1):(N+P)))';

DSx = DSx-mean(DSx);

m = 100;
Psum = zeros(m,m);
Ysum = zeros(m,1);
for i = 1:N
    phi = zeros(m,1);
    for j = 1:m
        if (i-j)>0
            phi(j,1) = -DSx(i-j);
        end
    end
    Psum = Psum + phi*phi';
    Ysum = Ysum + phi*DSx(i);
end
theta = inv(Psum)*Ysum;
ee = zeros(N,1);
for i = 1:N
    phi = zeros(m,1);
    for j = 1:m
        if (i-j)>0
            phi(j,1) = DSx(i-j);
        end
    end
    ee(i,1) = DSx(i) + sum(theta.*phi);
end

for p=0:10,
   for q=0:10,% 定义Dpq阵
        for i=1:N
            for j=1:(p+1)
                if (i-j)>=0
                    Dpq(i,j)=DSx(i-j+1);
                else
                    Dpq(i,j)=0;
                end
            end
            for j=(p+2):(p+q+2)
                if (i+p+1-j)>=0
                    Dpq(i,j)=ee(i-j+1+p+1);
                else
                    Dpq(i,j)=0;
                end
            end
        end
        Rpq=Dpq'*Dpq;
        RpqEig=eig(Rpq);
        RpqEigMin(p+1,q+1)=RpqEig(1);
    end
end
J = RpqEigMin';
for i = 1:10
    Jpq(i,:)=J(i+1,:)./J(i,:);
    Jpq1(:,i)=J(:,i+1)./J(:,i);
end

[RM,orderPQ]= min(Jpq,[],1);
[M,orderQ]= min(RM);
orderQ=orderPQ(orderQ)
[RM1,orderPQ1]= min(Jpq1,[],2);
[M1,orderQ1]= min(RM1);
orderP=orderPQ1(orderQ1)
t=1:N;
plot(t,DSx);xlabel('Samples'),ylabel('\it\rm{x}')