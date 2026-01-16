clear;clc;close all;
%% 参数设置
m = 100;
g= 9.8;
rho = 1.225;
S = 0.01;
C_Q = 1; C_Y = 0.2;
alpha = deg2rad(3);
t = 0; dt = 0.01;
X = [0,0,50,deg2rad(30)];
zero_count = 0;
solution = [];
%% 仿真
while zero_count ~= 2
    % 状态计算
    P = cal_P(t);
    [Q,Y] = cal_QY(X,rho,S,C_Q,C_Y);
    % 状态存储 
    solution = [solution;t,X,P];
    % 状态更新
    dX1 = cal_dX(X,Q,Y,m,P,alpha,g);
    dX2 = cal_dX(X + 0.5*dt*dX1,Q,Y,m,P,alpha,g);
    dX3 = cal_dX(X + 0.5*dt*dX2,Q,Y,m,P,alpha,g);
    dX4 = cal_dX(X + 0.5*dt*dX3,Q,Y,m,P,alpha,g);
    X = X + dt/6*(dX1 + 2*dX2 + 2*dX3 + dX4);
    t = t + dt;
    % 落地检测
    if X(2)<1e-1
        zero_count = zero_count + 1;
    end
end
%% 绘图
figure(1);
subplot(3,2,1);
hold on;grid on;
plot(solution(:,1),solution(:,2),"LineWidth",2);
xlabel("t(s)");ylabel("x(m)");
subplot(3,2,2);
hold on;grid on;
plot(solution(:,1),solution(:,3),"LineWidth",2);
xlabel("t(s)");ylabel("y(m)");
subplot(3,2,3);
hold on;grid on;
plot(solution(:,1),solution(:,4),"LineWidth",2);
xlabel("t(s)");ylabel("v(m/s)");
subplot(3,2,4);
hold on;grid on;
plot(solution(:,1),rad2deg(solution(:,5)),"LineWidth",2);
xlabel("t(s)");ylabel("theta(deg)");
subplot(3,2,5);
hold on;grid on;
plot(solution(:,1),solution(:,6),"LineWidth",2);
xlabel("t(s)");ylabel("P(N)");
figure(2);
hold on;grid on;
plot(solution(:,2),solution(:,3),"LineWidth",2);
xlabel("x");ylabel("y");
%% 函数
function P = cal_P(t)
    if t>=1 && t<=11
        P = 500;
    else
        P =0;
    end
end

function [Q,Y] = cal_QY(X,rho,S,C_Q,C_Y)
    x = X(1);
    y = X(2);
    v = X(3);
    theta = X(4);
    Q = 0.5*rho*(v^2)*S*C_Q;
    Y = 0.5*rho*(v^2)*S*C_Y;
end

function dX = cal_dX(X,Q,Y,m,P,alpha,g)
    x = X(1);
    y = X(2);
    v = X(3);
    theta = X(4);

    dx = v*cos(theta);
    dy = v*sin(theta);
    dv = -g*sin(theta) - Q/m + P*cos(alpha);
    dtheta =( -g*cos(theta) + Y/m + P*sin(alpha)/m )/v;

    dX = [dx,dy,dv,dtheta];
end