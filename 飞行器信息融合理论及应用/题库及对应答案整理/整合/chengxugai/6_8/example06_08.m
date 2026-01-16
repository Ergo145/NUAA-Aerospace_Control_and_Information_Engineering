% 有色噪声Kalman滤波器
%假设alpha-beta-gama滤波中的加加速度非白噪声而为一阶Markov过程
%进行状态扩维后滤波程序如下
%alpha-beta-gama滤波
%系统模型x[k+1]=Phi*x[k]+WN[k]
%噪声模型WN[k+1]=exp(-a*T)WN[k]+sigma*sqrt(1-exp(-2*a*T))*w[k]
%量测模型z[k]=H*x[k]+v[k]
clear all;
close all;
clc

%系统参数
T=0.2;%采样间隔
%噪声方差阵以及初值设置
Q=0.9;
sigma=sqrt(Q);
R=0.6;
I=eye(4);
N = 200;
a=0.11;

w = sigma*sqrt(1-exp(-2*a*T))*randn(N,1);
v = R^0.5*randn(N,1);

Phi=[1 T  0.5*T^2 1/6*T^3;0 1 T 0.5*T^2;0 0 1 T;0 0 0 exp(-a*T)];%状态转移矩阵
G=[0 0 0 1]';%加加速度为有色噪声
H=[1 0 0 0];%只对位置进行测量

xr(:,1)=zeros(4,1);
xr(4,1)=w(1,1);
for i = 2:N
    xr(:,i) = Phi*xr(:,i-1)+G*w(i,1);
    z(:,i)=H*xr(:,i)+v(i,1);
end

xe(:,1)=zeros(4,1);

Ppos=eye(4);
Ppre(:,1)=diag(Ppos);
Pest(:,1)=diag(Ppos);

for i=2:N
    %状态传递预测
    x(:,i)=Phi*xe(:,i-1);
    Pneg=Phi*Ppos*Phi'+G*Q*G';
    Ppre(:,i)=diag(Pneg);%提取对角元素

    %滤波计算（量测更新）过程
    K(:,i)=Pneg*H'*inv(H*Pneg*H'+R);
    Ppos=(I-K(:,i)*H)*Pneg;
    Pest(:,i)=diag(Ppos);%提取对角元素
    xe(:,i)=x(:,i)+K(:,i)*(z(:,i)-H*x(:,i));%状态估计值
end

Phi1=Phi(1:3,1:3);%状态转移矩阵
G1=[0 0 T]';%加加速度为有色噪声
H1=[1 0 0];%只对位置进行测量
I1 = eye(3);

xe1(:,1)=zeros(3,1);

Ppos1=eye(3);
Ppre1(:,1)=diag(Ppos1);
Pest1(:,1)=diag(Ppos1);

for i=2:N
    %状态传递预测
    x1(:,i)=Phi1*xe1(:,i-1);
    Pneg1=Phi1*Ppos1*Phi1'+G1*Q*G1';
    Ppre1(:,i)=diag(Pneg1);%提取对角元素

    %滤波计算（量测更新）过程
    K1(:,i)=Pneg1*H1'*inv(H1*Pneg1*H1'+R);
    Ppos1=(I1-K1(:,i)*H1)*Pneg1;
    Pest1(:,i)=diag(Ppos1);%提取对角元素
    xe1(:,i)=x1(:,i)+K1(:,i)*(z(:,i)-H1*x1(:,i));%状态估计值
end

pos_diff = xe(1,:)-xr(1,:);
pos_diff1 = xe1(1,:)-xr(1,:);

pos_diff_m = mean(pos_diff);
pos_diff_s = std(pos_diff);
pos_diff_m1 = mean(pos_diff1);
pos_diff_s1 = std(pos_diff1);

t=(1:N)*T;
plot(t,pos_diff,'b-',t,pos_diff1,'ro--'),
legend('状态扩展','近似为白噪声');
xlabel('时间(s)'), ylabel('位置误差(m)');