% Codes for Example 3-10
% The program is to generate the discrete sequence for a system.

clear all;
close all;

q = 0.1;
delta_t = 0.1;
N = 1000;
M = 5;

w_unit = randn(M,N); % zero mean unit white noise sequence 
q_k = sqrt(q*delta_t);
w_k = q_k*w_unit; % generate the discrete white noise with the variance of q_k

x(:,1) = [0 0 0 0 0]; % initial value of state x
for i = 2:N
    x(:,i) = x(:,i-1)+w_k(:,i-1);
end 

t = (1:N)*delta_t;
plot(t,x(1,:),'ko-',t,x(2,:),'kx-',t,x(3,:),'k*-',t,x(4,:),'k-.',t,x(5,:),'k');
xlabel('\it\fontname{Times New Roman}t\rm(s)');
ylabel('\it\fontname{Times New Roman}x\rm(\itt\rm)');grid