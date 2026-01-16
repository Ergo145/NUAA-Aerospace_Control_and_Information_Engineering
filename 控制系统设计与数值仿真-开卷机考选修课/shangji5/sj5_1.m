clc;clear;close all;
% X = [x,theta,dx,dtheta];
m1 = 20;
m2 = 10;
l = 5;
g = 9.8;
X = [10,deg2rad(30),0,deg2rad(0)];
dt = 0.01;
t = 0:dt:100;
kp = 0.1;
kd = 1;
soulution = [];
for k=1:length(t)
    F = cal_F(X,kp,kd,m2,m1,l,g);
    
    dX1 = cal_dX(X,m1,m2,l,F,g);
    dX2 = cal_dX(X + 0.5*dt*dX1,m1,m2,l,F,g);
    dX3 = cal_dX(X + 0.5*dt*dX2,m1,m2,l,F,g);
    dX4 = cal_dX(X + dt*dX3,m1,m2,l,F,g);

    X = X + (dt/6)*(dX1 + 2*dX2 + 2*dX3 + dX4);
    soulution = [soulution;X,F];
end
figure;
subplot(2,2,1);
hold on;grid on;
plot(t,soulution(:,1),"LineWidth",2);
xlabel("t(s)");ylabel("x(m)");
subplot(2,2,2);
hold on;grid on;
plot(t,rad2deg(soulution(:,2)),"LineWidth",2);
xlabel("t(s)");ylabel("theta(deg)");
subplot(2,2,3);
hold on;grid on;
plot(t,soulution(:,5),"LineWidth",2);
xlabel("t(s)");ylabel("F(N)");
%% 函数
function F = cal_F(X,kp,kd,m2,m1,l,g)
% X = [x,theta,dx,dtheta];
    x = X(1);
    theta = X(2);
    dx = X(3);
    dtheta = X(4);

    F = -m2*(g*cos(theta) + l*(dtheta^2))*sin(theta) ...
        -kp*(m1+m2*(sin(theta))^2)*(m1*x - m2*l*sin(theta))...
        -kd*(m1*dx - m2*l*dtheta*cos(theta));
end
function dX = cal_dX(X,m1,m2,l,F,g)
    x = X(1);
    theta = X(2);
    dx = X(3);
    dtheta = X(4);

    delta = m1 + m2*(sin(theta))^2;
    ddx = (F + m2*l*(dtheta^2)*sin(theta) + m2*g*sin(theta)*cos(theta))/delta;
    ddtheta = g*(m1 + m2)*sin(theta) + F*cos(theta) + m2*l*(dtheta^2)*sin(theta)*cos(theta);
    ddtheta = -ddtheta/l/delta;

    dX = [dx,dtheta,ddx,ddtheta];
end