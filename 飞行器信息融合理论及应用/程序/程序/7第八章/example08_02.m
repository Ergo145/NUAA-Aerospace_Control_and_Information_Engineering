% EG15 UKF滤波器 SUT+Redraw
% 阻尼单摆
% d(x1)/dt=x2;
% d(x2)/dt=-10*sin(x1)-x2;
% z1=2*sin(0.5*x1);z2=0.5*x1;
% 初始化
%
clear
clc
q1=0.01;
q2=0.0001;
r1=0.1;
r2=0.1;
T=0.05;
Q=diag([q1,q2]);
R=diag([r1,r2]);
SampleNo=200;
xestk=[1;0];
xrk=xestk+0.01*randn*[1;0.1];
Pestk=diag([1,1]);
xpre=[];xr=[];xest=[];zr = [];xhat = [];

zrk = [2*sin(0.5*xrk(1));0.5*xrk(1)]+[sqrt(r1)*randn;sqrt(r2)*randn];
xr = [xr xrk];
zr = [zr zrk];
for k = 2:SampleNo
    xrk=[xrk(1)+T*xrk(2);(1-T)*xrk(2)-10*T*sin(xrk(1))]+[sqrt(q1)*randn;sqrt(q2)*randn];
    zrk=[2*sin(0.5*xrk(1));0.5*xrk(1)]+[sqrt(r1)*randn;sqrt(r2)*randn];
    xr=[xr xrk];
    zr = [zr zrk];
end

xh = xestk;
P = Pestk;

%
%
alpha=0.1;
beta=2;
kappa=1;
L=2;%状态维数
lamda=alpha^2*(L+kappa)-L;
gama=sqrt(L+lamda);
% sigma点权重值设置(SUT,如需变化在此改动采样策略),这里生成的权值向量为行向量
Wm(1)=lamda/(L+lamda);
Wc(1)=lamda/(L+lamda)+1-alpha^2+beta;
for i=2:(2*L+1),
    Wm(i)=1/(2*(L+lamda));
    Wc(i)=1/(2*(L+lamda));
end

for k=1:SampleNo
    
    %EKF
    F = [1 T;-10*T*sin(xh(1)) 1-T];
    P = F * P * F' + Q;
    xh = [xh(1)+T*xh(2);(1-T)*xh(2)-10*T*sin(xh(1))];%预测
    H = [cos(xh(1)/2) 0;0.5 0];
    K = P * H' * (H * P * H' + R)^(-1);
    xh = xh + K * (zr(:,k) - H*xh);%更新
    P = (1 - K * H) * P;
    
    %UKF
    % sigma点的计算
    sqrtPk=utchol(Pestk);%这里Pak的平方根计算可以任意选择，比如可取cholesky分解，则可实现SR-UKF算法
    Xsigmak(:,1)=xestk;
    Xsigmak(:,2)=Xsigmak(:,1)+gama*sqrtPk(:,1);
    Xsigmak(:,3)=Xsigmak(:,1)+gama*sqrtPk(:,2);
    Xsigmak(:,4)=Xsigmak(:,1)-gama*sqrtPk(:,1);
    Xsigmak(:,5)=Xsigmak(:,1)-gama*sqrtPk(:,2);
    % Time-update
    Xsigmakf=zeros(2,5);
    for i=1:5
        Xsigmakf(:,i)=[Xsigmak(1,i)+T*Xsigmak(2,i);(1-T)*Xsigmak(2,i)-10*T*sin(Xsigmak(1,i))];
    end;
    %系统状态转移函数，根据具体情况编制，注意要求输入中Xak为L*（2L+1）的矩阵，uk为控制向量，输出为dimx*（2L+1）的矩阵
    xkpre=Xsigmakf*Wm';%均值预测
    Pkpre=Q;
    for i=1:(2*L+1),
        Pkpre=Pkpre+Wc(i)*(Xsigmakf(:,i)-xkpre)*(Xsigmakf(:,i)-xkpre)';
    end;
    Zsigmakh=zeros(2,5);
    for i=1:5
        Zsigmakh(:,i)=[2*sin(0.5*Xsigmak(1,i));0.5*Xsigmak(1,i)];
    end;
    zkpre=Zsigmakh*Wm';%量测预测
    Pzzk=R;
    Pxzk=zeros(2);
    for i=1:(2*L+1),
        Pzzk=Pzzk+Wc(i)*(Zsigmakh(:,i)-zkpre)*(Zsigmakh(:,i)-zkpre)';
        Pxzk=Pxzk+Wc(i)*(Xsigmak(:,i)-xkpre)*(Zsigmakh(:,i)-zkpre)';
    end
    % 量测更新
    vnewk=zr(:,k)-zkpre;
    K=Pxzk/Pzzk;
    xestk=xkpre+K*vnewk;
    Pestk=Pkpre-K*Pzzk*K';
    %保存数据
    xpre=[xpre xkpre];
    xest=[xest xestk];
    xhat = [xhat xh];
end
%作图分析环节
t=(1:SampleNo)*T;
figure(1);
% subplot(211),
plot(t,xr(1,:),'-',t,xr(2,:),'o-'),
legend('状态1实际值','状态2实际值');
xlabel('时间（s）'), ylabel('状态真值');
% subplot(212),
% plot(t,xr(2,:),'-',t,xpre(2,:),'-.',t,xest(2,:),'--'),
% legend('实际值','预测值','滤波值');
% xlabel('采样数'), ylabel('状态2');figure(2);
figure(2)
plot(t,xr(1,:),'k-',t,xpre(1,:),'k-.',t,xest(1,:),'k*-',t,xr(2,:),'ko-',t,xpre(2,:),'ks-',t,xest(2,:),'kv-'),
legend('状态1实际值','状态1预测值','状态1滤波值','状态2实际值','状态2预测值','状态2滤波值');
xlabel('时间（s）'), ylabel('状态值');
figure(3)
plot(t,xr(1,:)-xpre(1,:),'k-',t,xr(1,:)-xest(1,:),'ko-',t,xr(2,:)-xpre(2,:),'k*-',t,xr(2,:)-xest(2,:),'ks-'),
legend('状态1预测误差','状态1滤波误差','状态2预测误差','状态2滤波误差');
xlabel('时间（s）');ylabel('滤波误差');
figure(4)
plot(t,xr(1,:)-xest(1,:),'k-',t,xr(1,:)-xhat(1,:),'ko-',t,xr(2,:)-xest(2,:),'k*-',t,xr(2,:)-xhat(2,:),'ks-'),
legend('状态1滤波误差（UKF）','状态1滤波误差（EKF）','状态2滤波误差（UKF)','状态2滤波误差(EKF)');
xlabel('时间（s）');ylabel('滤波误差');