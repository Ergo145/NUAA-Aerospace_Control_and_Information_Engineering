%l例程14  EKF一阶滤波器
% 系统模型
% x[k+1]=f(x[k])+w[k]
% y[k]=h(x[k])+v[k]
%
% 涉及项：
% f   状态转移函数
% F   状态函数f在状态预测值处的一阶Taylor展开对应的Jacobian阵
% Q   噪声方差阵
% h   量测函数
% H   量测函数h在状态预测值处的一阶Taylor展开对应的Jacobian阵
% R   量测噪声方差阵
% x0  初始状态值
% P0  初始估计误差阵
% SampleNo 采样数
%
% X1[K+1]=X2[K]*sin(X1[K])+0.1*K
% X2[K+1]=X1[K]+cos(X2[K])^2-0.1*K
% Z1[K]=sqrt(X1[K]^2+X2[K]^2);Z2[K]=arctan(X1[K]/X2[K])
% 初始化
%
clear
clc
q1=0.01;
q2=0.1;
r1=1;
r2=0.1;
Q=diag([q1,q2]);
R=diag([r1,r2]);
SampleNo=500;
xhat=[1;1];%x(0)
Pest=diag([10,10]);
xpre=[];xr=[];xest=[];zr = [];

xrk = xhat;%+[sqrt(q1)*randn;sqrt(q2)*randn];
% zrk=[sqrt(xrk(1)^2+xrk(2)^2);atan(xrk(1)/xrk(2))]+[sqrt(r1)*randn;sqrt(r2)*randn];
% xr = [xr xrk];
% zr = [zr zrk];
%
for k = 1:SampleNo
    xrk = [sin(xrk(1))*xrk(2)+0.1*k;cos(xrk(2))^2+xrk(1)-0.1*k]+[sqrt(q1)*randn;sqrt(q2)*randn];
    zrk = [sqrt(xrk(1)^2+xrk(2)^2);atan(xrk(1)/xrk(2))]+[sqrt(r1)*randn;sqrt(r2)*randn];
    xr = [xr xrk];
    zr = [zr zrk];
end
%
% xhat = xr(:,1);
for k=1:SampleNo
    %调用预测过程
    xprek=[sin(xhat(1))*xhat(2)+0.1*k;cos(xhat(2))^2+xhat(1)-0.1*k];
    Phi=[xhat(2)*cos(xhat(1)),sin(xhat(1));1,-sin(2*xhat(2))];
    Pprek=Phi*Pest*Phi'+Q;
    zpre=[sqrt(xprek(1)^2+xprek(2)^2);atan(xprek(1)/xprek(2))];
    H=[xprek(1)/sqrt(xprek(1)^2+xprek(2)^2),xprek(2)/sqrt(xprek(1)^2+xprek(2)^2);xprek(2)/(xprek(1)^2+xprek(2)^2),-xprek(1)/(xprek(1)^2+xprek(2)^2)];
    %调用量测处理函数
    zz = zr(:,k);
    vnewk=zz-zpre;
    Pvnewk=H*Pprek*H'+R;
    Pxzk=Pprek*H';
    %调用滤波估计函数
    K=Pxzk/Pvnewk;
    xhat=xprek+K*vnewk;
    Pest=Pprek-K*Pvnewk*K';
%     Pest=0.5*(Pest+Pest');
    %调用数据记录函数
    xpre=[xpre xprek];
    xest=[xest xhat];
end
%作图分析环节
t=1:SampleNo;
figure(1)
plot(t,xr(1,1:SampleNo),'o-',t,xr(2,1:SampleNo),'-')
legend('真值（状态1）','真值（状态2）');
xlabel('采样点'),ylabel('状态真值')
figure(2);
plot(t,xr(1,1:SampleNo),'-',t,xpre(1,:),'o-',t,xest(1,:),'*-'),
legend('实际值','预测值','滤波值');
xlabel('采样点'), ylabel('状态1');
figure(3),
plot(t,xr(2,1:SampleNo),'-',t,xpre(2,:),'o-',t,xest(2,:),'*-'),
legend('实际值','预测值','滤波值');
xlabel('采样点'), ylabel('状态2');
figure(4);
plot(t,xr(1,1:SampleNo)-xest(1,:),'-',t,xr(1,1:SampleNo)-xpre(1,:),'o-',t,xr(2,1:SampleNo)-xest(2,:),'--',t,xr(2,1:SampleNo)-xpre(2,:),'*-'),
legend('状态1滤波误差','状态1预测误差','状态2滤波误差','状态2预测误差');
xlabel('采样点');ylabel('滤波误差');