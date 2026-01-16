%连续系统离散化例程
%二阶Markov过程
%本例为连续系统的处理以及函数模块调用示例
%离散化后用标准kalman滤波得到结果
%系统微分方程：d^2x/dt^2+2*ksi*omega*dx/dt+omega^2*x=w
%状态空间方程：dX/dt=FX+Gw
clear all;
close all;
%参数设置:%
T     = 0.01;  % 采样间隔 
Qc    = 9.9;   % 连续过程噪声方差
R     = 0.1;   % 量测噪声方差
ksi   = 0.707; % 阻尼系数
omega = 2;     % 阻尼共振角频率(rad/s)
H     = [1,0]; % 量测矩阵
F     = [0,1;-omega^2,-2*ksi*omega];
beta  = omega*sqrt(1-ksi^2);
% 一步转移矩阵Phi=e^(T*F)采用
% 矩阵指数expm(T*F)指令计算%
Phi=expm(T*F);
% 离散过程噪声协方差阵计算，计算方法参考相关章节
% 本例数据结果参见习题3.9。由于模型系数不同，此处系数略有差别，为原系数乘以omega^（-4）
Qd(1,1)=(0.25*Qc*omega^(-3)/ksi)* (1-exp(-2*omega*ksi*T)*beta^(-2)*(omega^2-(ksi*omega)^2*cos(2*beta*T)+ksi*omega*beta*sin(2*beta*T)));
Qd(1,2)=0.5*Qc*omega^(-2)*exp(-2*omega*ksi*T)*(sin(beta*T))^2/(1-ksi^2);
Qd(2,1)=Qd(1,2);
Qd(2,2)=(0.25*Qc*omega^(-1)/ksi)* (1-exp(-2*omega*ksi*T)*beta^(-2)*(omega^2-(ksi*omega)^2*cos(2*beta*T)-ksi*omega*beta*sin(2*beta*T)));
% 初始值滤波设置设置
x0=zeros(2,1);
P0=3*eye(2);
SampleNo=100;
xrkdelay=rand(2,1);
% 调用初始化函数
[Q,xestkdelay,Pestkdelay,N,Ppre,xpre,xr,Kgain,xest,Pest]=DKFinitial(Qd,x0,P0,SampleNo);
for k=1:N
    %时间设置，如果采用采样数做横坐标，则在作图环节自设即可，无需此步
    t(:,k)=(k-1)*T;
    %调用预测函数
    [xprek,zprek,Pprek]=Pred(Phi,xestkdelay,Pestkdelay,H,Q);
    %调用量测处理函数
    [vnewk,Pvnewk,Pxzk,xrk]=Innova(Phi,xrkdelay,Q,zprek,H,Pprek,R);
    %调用滤波估计函数
    [xestk,K,Pestk] = Update(Pprek,Pvnewk,Pxzk,xprek,vnewk);
    %调用数据记录函数
    [Ppre,xpre,xr,Kgain,xest,Pest]=recorda(Ppre,xpre,xr,Kgain,xest,Pest,Pprek,xprek,K,xestk,Pestk,xrk);
    %调用延迟函数，为下次循环做准备
    [xestkdelay,Pestkdelay,xrkdelay]=Kdelaya(xestk,Pestk,xrk);
end
%
% 作图分析环节
%
figure(1);
plot(t,xpre(1,:),'-',t,xr(1,:),'bo-',t,xest(1,:),'r*-'),
legend('预测值','实际值','滤波值');
xlabel('时间'), ylabel('位置');
figure(2)
plot(t,xpre(2,:),'-',t,xr(2,:),'bo-',t,xest(2,:),'r*-'),
legend('预测值','实际值','滤波值');
xlabel('时间'), ylabel('速度');
figure(3);
plot(t,xpre(1,:)-xr(1,:),'-',t,xest(1,:)-xr(1,:),'bo-',t,xpre(2,:)-xr(2,:),'r*-',t,xest(2,:)-xr(2,:),'k--'),
legend('位置预测误差','位置滤波误差','速度预测误差','速度滤波误差');
xlabel('时间'), ylabel('估计误差');
figure(4)
plot(t,Ppre(1,:),'k*-',t,Ppre(2,:),'ko-',t,Pest(1,:),'kv--',t,Pest(2,:),'ks--'),legend('位置预测','速度预测','位置估计','速度估计');
xlabel('时间');ylabel('P对角线元素');
figure(5)
plot(t,Kgain(1,:),'k*-',t,Kgain(2,:),'ko-'),legend('位置增益','速度增益');
xlabel('时间)');ylabel('滤波增益');