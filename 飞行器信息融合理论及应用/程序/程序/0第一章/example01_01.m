% 第一章绪论例1-4
% 编写人：王可东
% 编写时间：2016.9.10

clear all;
close all;

N = 100;
x = 50 * ones(N,1);  %被估计对象
sigma1 = 0.1;
sigma2 = 0.2;
v1 = sigma1 * randn(N,1);
v2 = sigma2 * randn(N,1);
z1 = x + v1;
z2 = x + v2;

w1 = sigma1^2/(sigma1^2 + sigma2^2);
w2 = sigma2^2/(sigma1^2 + sigma2^2);
x_est = w1 * z2 + w2 * z1;

figure(1)
plot(1:N,x_est,'k*-',1:N,x,'k',1:N,z1,'ks-',1:N, z2,'ko-');
legend('估计值','真值','测量值1','测量值2');
xlabel('时间（s）');ylabel('速度（m/s）')

figure(2)
plot(1:N,x_est-x,'k*-',1:N,v1,'k',1:N,v2,'ko-');
legend('估计误差','测量误差1','测量误差2')
xlabel('时间（s）');ylabel('速度误差（m/s）')