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
I=eye(3);
N = 200;
a=0.11;

w = sigma*randn(N,1);
pusi = sqrt(R)*sqrt(1-exp(-2*a*T))*randn(N,1);
Ps = exp(-a*T);

v = zeros(N,1);
v(1,1) = pusi(1,1);
for i = 2:N
    v(i,1) = Ps*v(i-1,1)+pusi(i,1);
end

Phi=[1 T  0.5*T^2;0 1 T;0 0 1];%状态转移矩阵
G=[0 0 T]';%加加速度为有色噪声
H=[1 0 0];%只对位置进行测量

xr(:,1)=zeros(3,1);
xr(3,1)=w(1,1);
for i = 2:N
    xr(:,i) = Phi*xr(:,i-1)+G*w(i,1);
    z(:,i)=H*xr(:,i)+v(i,1);
end

Qtemp = G*Q*G';
R_star = H*Qtemp*H'+R;
J = Qtemp*H'*inv(R_star);
H_star = H*Phi-Ps*H;
Phi_star = Phi-J*H_star;
Q_star = Qtemp-Qtemp*H'*inv(R_star)*H*Qtemp;

for i=1:N-1
    z_star(:,i) = z(:,i+1)-Ps*z(:,i);
end

xe(:,1)=zeros(3,1);

Ppos=eye(3);
Ppre(:,1)=diag(Ppos);
Pest(:,1)=diag(Ppos);

xe(:,1) = xe(:,1)+Ppos*H'*inv(H*Ppos*H'+R)*(z(:,1)-H*xe(:,1));
Ppos = inv(inv(Ppos)+H'*inv(R)*H);

for i=2:N-1
    %状态传递预测
    x(:,i)=Phi_star*xe(:,i-1)+J*z_star(:,i-1);
    Pneg=Phi_star*Ppos*Phi_star'+Q_star;
    Ppre(:,i)=diag(Pneg);%提取对角元素

    %滤波计算（量测更新）过程
    K(:,i)=Pneg*H_star'*inv(H_star*Pneg*H_star'+R_star);
    Ppos=(I-K(:,i)*H_star)*Pneg;
    Pest(:,i)=diag(Ppos);%提取对角元素
    xe(:,i)=x(:,i)+K(:,i)*(z_star(:,i)-H_star*x(:,i));%状态估计值
end

xe1(:,1)=zeros(3,1);

Ppos1=eye(3);
Ppre1(:,1)=diag(Ppos1);
Pest1(:,1)=diag(Ppos1);
R1 = R*(1-exp(-2*a*T));

for i=2:N-1
    %状态传递预测
    x1(:,i)=Phi*xe1(:,i-1);
    Pneg1=Phi*Ppos1*Phi'+G*Q*G';
    Ppre1(:,i)=diag(Pneg1);%提取对角元素

    %滤波计算（量测更新）过程
    K1(:,i)=Pneg1*H'*inv(H*Pneg1*H'+R1);
    Ppos1=(I-K1(:,i)*H)*Pneg1;
    Pest1(:,i)=diag(Ppos1);%提取对角元素
    xe1(:,i)=x1(:,i)+K1(:,i)*(z(:,i)-H*x1(:,i));%状态估计值
end

pos_diff = xe(1,:)-xr(1,1:N-1);
pos_diff1 = xe1(1,:)-xr(1,1:N-1);

pos_diff_m = mean(pos_diff);
pos_diff_s = std(pos_diff);
pos_diff_m1 = mean(pos_diff1);
pos_diff_s1 = std(pos_diff1);

t=(1:N-1)*T;
plot(t,pos_diff,'b',t,pos_diff1,'ro--'),
legend('状态扩展','近似为白噪声');
xlabel('时间(s)'), ylabel('位置误差(m)');