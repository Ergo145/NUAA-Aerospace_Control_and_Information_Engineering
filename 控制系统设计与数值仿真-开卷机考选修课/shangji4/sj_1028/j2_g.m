clc;clear;

%% 初始化
m = 100;
g = 9.8;
rho = 1.225;
S = 0.01;
C_Q = 1;
C_Y = 0.2;
alpha = deg2rad(3);
P = 0;  % 初始推力为0
Z = [0,0,50,deg2rad(30)];
dt = 0.01;
t=0;
zero_count=0;
flag = true;
Zout = [];
Tout = [];  % 新增时间记录数组
%% 仿真
while flag
    % 修改推力控制逻辑：仅在1-11秒时为500
    if t >= 1 && t < 11
        P = 500;
    else
        P = 0;
    end
    
    dZ1 = calculate_flight(Z,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ2 = calculate_flight(Z + 0.5*dt*dZ1,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ3 = calculate_flight(Z + 0.5*dt*dZ2,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ4 = calculate_flight(Z + dt*dZ3,P,m,g,alpha,rho,S,C_Q,C_Y);

    Z = Z + (dZ1 + 2*dZ2 + 2*dZ3 + dZ4)*dt/6;
    Zout = [Zout;Z];
    Tout = [Tout; t];  % 记录当前时间
    t = t+dt;
    if Z(2) < 1e-1
        zero_count=zero_count+1;
    end
    if zero_count > 1
        flag = false;
    end
end
%% 绘图
% 修正时间向量和状态向量的构造
Zout = [0,0,50,deg2rad(30); Zout];  % 添加初始状态
Tout = [0; Tout];  % 添加初始时间

figure;
title("发动机参数推力曲线")
subplot(3,2,1);
hold on;
grid on;
plot(Tout,Zout(:,1),'b-','LineWidth',2);  % 使用Tout而不是tf
xlabel("t(s)");ylabel("x(m)");

subplot(3,2,2);
hold on;
grid on;
plot(Tout,Zout(:,2),'b-','LineWidth',2);  % 使用Tout而不是tf
xlabel("t(s)");ylabel("y(m)");

subplot(3,2,3);
hold on;
grid on;
plot(Tout,Zout(:,3),'b-','LineWidth',2);  % 使用Tout而不是tf
xlabel("t(s)");ylabel("v(m/s)");

subplot(3,2,4);
hold on;
grid on;
plot(Tout,rad2deg(Zout(:,4)),'b-','LineWidth',2);  % 使用Tout而不是tf
xlabel("t(s)");ylabel("theta(deg)");

subplot(3,2,5);
hold on;
grid on;
% 修改推力绘图部分
p = zeros(size(Tout));
for i = 1:length(Tout)
    if Tout(i) >= 1 && Tout(i) < 11
        p(i) = 500;
    else
        p(i) = 0;
    end
end
plot(Tout,p,'b-','LineWidth',2);
xlabel("t(s)");ylabel("P(N)");

subplot(3,2,6);
hold on;
grid on;
plot(Zout(:,1),Zout(:,2),'b-','LineWidth',2)
xlabel("x");ylabel("y");

%% 函数
% 飞行力学
% Z = [x y v theta]
function dZ = calculate_flight(Z,P,m,g,alpha,rho,S,C_Q,C_Y)
    x = Z(1);
    y = Z(2);
    v = Z(3);
    theta = Z(4);
    [Q,Y] = calculate_qidong(v,rho,S,C_Q,C_Y);
    dx = v*cos(theta);
    dy = v*sin(theta);
    dv = -g*sin(theta) - Q/m + P*cos(alpha);
    dtheta = (-g*cos(theta) + Y/m + P*sin(alpha)/m)/v;
    
    dZ = [dx,dy,dv,dtheta];
end
% 气动参数计算
function [Q,Y] = calculate_qidong(v,rho,S,C_Q,C_Y)
    Q = 0.5*rho*(v^2)*S*C_Q;
    Y = 0.5*rho*(v^2)*S*C_Y;
end