clear all;
close all;

T=0.1;
a=2e-4;%a=2;
k=0.5;
rk1=0.1;
rk2=1;
N=1000;

phik=exp(-a*T);
Cx=k*(1-exp(-2*a*T));
qk=sqrt(Cx);

wn=randn(N,1);
Wk=qk*wn;

xk=zeros(N,1);
xk(1,1)=Wk(1,1);
for i=2:N
    xk(i,1)=phik*xk(i-1,1)+Wk(i,1);
end

vn=randn(N,1);
vk1=rk1*vn;
zk1=xk+vk1;

vn=randn(N,1);
vk2=rk2*vn;
zk2=xk+vk2;

h=[1;1];
R=[rk1^2 0;0 rk2^2];
zk=[zk1,zk2];
R_inv=inv(R);

% Weighted Iterative Least Squares
wk=R_inv;
x_est_wls(1,1)=0;
p_est_wls(1)=1e6;
p_est_wls(1)=1/(1/p_est_wls(1)+h'*wk*h);
x_est_wls(1,1)=x_est_wls(1,1)+p_est_wls(1)*h'*wk*(zk(1,:)'-h*x_est_wls(1,1));
for i=2:N
    p_est_wls(i)=1/(1/p_est_wls(i-1)+h'*wk*h);
    x_est_wls(i,1)=x_est_wls(i-1,1)+p_est_wls(i)*h'*wk*(zk(i,:)'-h*x_est_wls(i-1,1));
end

% Minimum Variance
Kk=Cx*h'*inv(h*Cx*h'+R);
for i=1:N
    x_est_mv(i,1)=Kk*zk(i,:)';
end

x_est_mv1(1,1)=0;
p_est_mv1(1)=1e6;
p_est_mv1(1)=1/(1/Cx+h'*R_inv*h);
x_est_mv1(1,1)=x_est_mv1(1,1)+p_est_mv1(1)*h'*R_inv*(zk(1,:)'-h*x_est_mv1(1,1));
for i=2:N
    p_est_mv1(i)=inv(inv(p_est_mv1(i-1))+h'*R_inv*h);
    x_est_mv1(i,1)=x_est_mv1(i-1,1)+p_est_mv1(i)*h'*R_inv*(zk(i,:)'-h*x_est_mv1(i-1,1));
end

t=(0:N-1)*T;
figure(1)
plot(t,xk,'b-',t,x_est_wls,'r*-',t,x_est_mv1,'ko-');
xlabel('时间(s)');ylabel('估计值');
legend('真值','加权递推最小二乘估计','最小方差估计');

figure(2)
plot(t,x_est_wls-xk,'r*-',t,x_est_mv1-xk,'ko-');
xlabel('时间(s)');ylabel('估计偏差');
legend('加权递推最小二乘估计误差','最小方差估计误差');

figure(3)
plot(t,p_est_wls,'r*-',t,p_est_mv1,'ko-');
xlabel('时间(s)');ylabel('P');
legend('加权递推最小二乘估计','最小方差估计');