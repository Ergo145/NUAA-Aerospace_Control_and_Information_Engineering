%一个标准的Kalman滤波器
%alpha-beta-gama滤波
%系统模型x[k+1]=Phi*x[k]+G*w[k]
%量测模型z[k]=H*x[k]+v[k]
clear all;
close all;
%系统参数
T=0.2;%采样间隔
Phi=[1 T  0.5*T^2;0 1 T;0 0 1];%状态转移矩阵
G=[0 0 T]';%加加速度为白噪声
H=[1 0 0];%只对位置进行测量
%噪声方差阵以及初值设置
Q=0.1;
R=0.04;
I=eye(3);
xr(:,1)=randn(3,1);%假设状态真实值
xe(:,1)=zeros(3,1);%初始状态
Ppos=eye(3);%初始方差
Ppre(:,1)=diag(Ppos);
Pest(:,1)=diag(Ppos);
for i=2:100
    %状态预测
    x(:,i)=Phi*xe(:,i-1);
    Pneg=Phi*Ppos*Phi'+G*Q*G';
    Ppre(:,i)=diag(Pneg);%提取对角元素
    %过程噪声附加
    w=Q^0.5*randn;
    xr(:,i)=Phi*xr(:,i-1)+G*w;%实际状态值
    %量测噪声附加
    v=Q^0.5*randn;
    z(:,i)=H*xr(:,i)+v; %成量测值
    %滤波计算（量测更新）过程
    K(:,i)=Pneg*H'*inv(H*Pneg*H'+R);
    Ppos=(I-K(:,i)*H)*Pneg*(I-K(:,i)*H)'+K(:,i)*R*K(:,i)';
    Pest(:,i)=diag(Ppos);%提取对角元素
    xe(:,i)=x(:,i)+K(:,i)*(z(:,i)-H*x(:,i));%状态估计值
end
Ks=Ppos*H'*1/R;
xe1(:,1)=zeros(3,1);
for i=2:100
    xe1(:,i)=Phi*xe1(:,i-1)+Ks*(z(:,i)-H*Phi*xe1(:,i-1));
end

t=T*(1:100);
figure(1);
plot(t,abs(x(1,:)-xr(1,:)),'k',t,abs(xe(1,:)-xr(1,:)),'ro-',t,abs(xe1(1,:)-xr(1,:)),'b*-'),
legend('预测值误差','滤波误差','稳态增益滤波误差');
xlabel('时间'), ylabel('位移估计误差');
figure(2)
plot(t,abs(x(2,:)-xr(2,:)),'k',t,abs(xe(2,:)-xr(2,:)),'ro-',t,abs(xe1(2,:)-xr(2,:)),'b*-'),
legend('预测误差','滤波误差','稳态增益滤波误差');
xlabel('时间'), ylabel('速度估计误差');
figure(3)
plot(t,abs(x(3,:)-xr(3,:)),'k',t,abs(xe(3,:)-xr(3,:)),'ro-',t,abs(xe1(3,:)-xr(3,:)),'b*-'),
legend('预测误差','滤波误差','稳态增益滤波误差');
xlabel('时间'), ylabel('加速度估计误差');
figure(4);
plot(t,Ppre(1,:),'k*--',t,Ppre(2,:),'ko--',t,Ppre(3,:),'k--',t,Pest(1,:),'ks-',t,Pest(2,:),'kv-',t,Pest(3,:),'k-'),legend('p11(-)','p22(-)','p33(-)','p11(+)','p22(+)','p33(+)');
xlabel('时间');ylabel('P阵对角线元素');
figure(5)
plot(t,K(1,:),'k-',t,K(2,:),'k*-',t,K(3,:),'kv-'),legend('位移增益','速度增益','加速度增益');
xlabel('时间');ylabel('滤波增益');
%结果的简单说明
%系统实际上是由白噪声（加加速度）驱动，其中加速度就表现为随机游走特性
%本例中P阵实际上提取了估计误差阵Ppos的对角线元素