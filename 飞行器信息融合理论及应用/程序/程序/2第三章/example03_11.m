% Codes for Example 3-10
% The program is to generate the discrete sequence for a system.

clear all;
close all;

T = 1;
K = 0.5;
q = 0.1;
delta_t =0.1;
N = 1000;

w_unit = randn(1,N); % zero mean unit white noise sequence 

q_k = sqrt(q*K^2*(1-exp(-2*delta_t/T))/2/T);% variance of the discrete white noise
w_k = q_k*w_unit; % generate the discrete white noise with the variance of q_k

phi_k = exp(-delta_t/T);
x(1) = 1 + w_k(1); % initial value of state x
for i = 2:N
    x(i) = phi_k*x(i-1)+w_k(i);
end 

x_est(1) = 1;
p(1) = (x(1)-x_est(1))^2;
for i = 2:N
    x_est(i) = phi_k*x_est(i-1);
    p(i) = phi_k^2*p(i-1)+q_k^2;
end

x_est1(1) = 0.5;
p1(1) = (x(1)-x_est1(1))^2;
for i = 2:N
    x_est1(i) = phi_k*x_est1(i-1);
    p1(i) = phi_k^2*p1(i-1)+q_k^2;
end

figure(1)
t = (1:N)*delta_t;
plot(t,x,'r--',t,x_est,'bo-',t,x_est1,'k-');
xlabel('\it\fontname{Times New Roman}t\rm(s)');
ylabel('\it\fontname{Times New Roman}x')
lg = legend('$$\it{x}$$','$$\hat{x}$$','$${\hat{x}}_{1}$$');
set(lg,'interpreter','latex','Fontsize',12);

figure(2)
plot(t,sqrt(p),'bo-',t,sqrt(p1),'k-');
xlabel('\it\fontname{Times New Roman}t\rm(s)');
ylabel('\it\fontname{Times New Roman}P')
legend('\it\fontname{Times New Roman}x\rm(0)=1','\it\fontname{Times New Roman}x\rm(0)=0.5');