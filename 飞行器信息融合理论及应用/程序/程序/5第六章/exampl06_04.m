close all
clear all
clc

N = 7000;
AR = 0.2;
MA = 0.5;
sigmau = 1;
P = 1;
Q = 1;

u=normrnd(0,sigmau,N,1);
u=[zeros(P,1);u];
X=zeros(P,1);
MAT=[1,-MA];
MAF=fliplr(MAT);
ARF=fliplr(AR);%翻转向量使之与数据的增长方向一致
for i=(P+1):(N+P),
    U=u((i-Q):i);%更新U
    x(i)=ARF*X+MAF*U;
    X=[X(2:P);x(i)];%更新X
end
DSx=(x((P+1):(N+P)))';
% DSx = DSx-mean(DSx);

Hq = 30;
uw = u(2:N+1,1);
DSx_ma = zeros(N,1);
for i = 1:N
    ef = 0;
    for j = 1:Hq
        if i>j
            ef = ef + (AR-MA)*AR^(j-1)*uw(i-j,1);
         end
    end
    DSx_ma(i,1) = uw(i,1)+ef;
end
d_DSx = DSx-DSx_ma;
mean(d_DSx)
std(d_DSx)

Hp = 30;
DSx_ar = zeros(N,1);
for i = 1:N
    ef = 0;
    for j = 1:Hp
        if i>j
            ef = ef + (AR-MA)*MA^(j-1)*DSx_ar(i-j);
        end
    end
    DSx_ar(i,1) = uw(i,1) + ef;
end
dd_DSx = DSx-DSx_ar;
mean(dd_DSx)
std(dd_DSx)
plot(1:N,d_DSx,'b',1:N,dd_DSx,'r')
xlabel('Samples'),ylabel('\it{x}')