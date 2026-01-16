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

x_est_ls_1 = [];
x_est_ls_2 = [];
sum_1 = 0;
sum_2 = 0;
for i = 1:N
    sum_1 = sum_1 + z1(i);
    sum_2 = sum_2 + z2(i);
    est_1 = sum_1/i;
    est_2 = (sum_1 + sum_2)/(2*i);
    x_est_ls_1 = [x_est_ls_1;est_1];
    x_est_ls_2 = [x_est_ls_2;est_2];
end
x_est_err = x_est-x;
x_est_ls_1_err = x_est_ls_1 - x;
x_est_ls_2_err = x_est_ls_2 - x;

plot(1:N,x_est_err,'k',1:N,x_est_ls_1_err,'k*-',1:N,x_est_ls_2_err,'ko-')
legend('最小方差估计误差','最小二乘估计误差(单测量)','最小二乘估计误差(双测量）');
xlabel('时间(s)');ylabel('速度误差(m/s)')