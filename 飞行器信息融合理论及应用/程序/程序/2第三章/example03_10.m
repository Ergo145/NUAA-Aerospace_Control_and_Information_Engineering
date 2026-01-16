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
x(1) = 0; % initial value of state x
for i = 2:N
    x(i) = phi_k*x(i-1)+w_k(i-1);
end 

t = (1:N)*delta_t;
plot(t,x');
xlabel('\it\fontname{Times New Roman}t\rm(s)');
ylabel('\it\fontname{Times New Roman}x\rm(\itt\rm)')