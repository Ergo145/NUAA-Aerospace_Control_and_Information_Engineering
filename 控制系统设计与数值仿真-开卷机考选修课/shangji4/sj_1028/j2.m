clc;clear;

%% 初始化
m = 100;
g = 9.8;
rho = 1.225;
S = 0.01;
C_Q = 1;
C_Y = 0.2;
alpha = deg2rad(3);
P = 500;
Z = [0,0,50,deg2rad(30)];
dt = 0.01;
t=0;
zero_count=0;
flag = true;
Zout = [];
%% 仿真
while flag
    if t>=10
        P=0;
    end
    dZ1 = calculate_flight(Z,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ2 = calculate_flight(Z + 0.5*dt*dZ1,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ3 = calculate_flight(Z + 0.5*dt*dZ2,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ4 = calculate_flight(Z + dt*dZ3,P,m,g,alpha,rho,S,C_Q,C_Y);

    Z = Z + (dZ1 + 2*dZ2 + 2*dZ3 + dZ4)*dt/6;
    Zout = [Zout;Z];
    t = t+dt;
    if Z(2) < 1e-1
        zero_count=zero_count+1;
    end
    if zero_count > 1
        flag = false;
    end
end
%% 绘图
Zout=[0,0,50,deg2rad(30);Zout];
tf = 0:dt:t;
tf = [tf,t];
figure;
title("发动机参数推力曲线")
subplot(3,2,1);
hold on;
grid on;
plot(tf,Zout(:,1),'b-','LineWidth',2);
xlabel("t(s)");ylabel("x(m)");
subplot(3,2,2);
hold on;
grid on;
plot(tf,Zout(:,2),'b-','LineWidth',2);
xlabel("t(s)");ylabel("y(m)");
subplot(3,2,3);
hold on;
grid on;
plot(tf,Zout(:,3),'b-','LineWidth',2);
xlabel("t(s)");ylabel("v(m/s)");
subplot(3,2,4);
hold on;
grid on;
plot(tf,rad2deg(Zout(:,4)),'b-','LineWidth',2);
xlabel("t(s)");ylabel("theta(deg)");
subplot(3,2,5);
hold on;
grid on;
p=[];
for m=dt:dt:10
    p=[p,500];
end
for m=10:dt:t
    p=[p,0];
end
p=[p,0];
p(1)=0;
plot(tf,p,'b-','LineWidth',2);
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
    %dtheta = dtheta/v;
    
    dZ = [dx,dy,dv,dtheta];
end
% 气动参数计算
function [Q,Y] = calculate_qidong(v,rho,S,C_Q,C_Y)
    Q = 0.5*rho*(v^2)*S*C_Q;
    Y = 0.5*rho*(v^2)*S*C_Y;
end
