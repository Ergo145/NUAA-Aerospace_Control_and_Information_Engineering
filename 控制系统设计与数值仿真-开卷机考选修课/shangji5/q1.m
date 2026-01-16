clc;clear;
%% 参数
m1 = 20;
m2 = 10;
l = 5;
g = 9.8;
X = [10,0,deg2rad(30),0];
kp = 0.1;
kd = 1;
xout = [];
fout = [];
tmax = 100;
dt = 0.01;
%% 仿真
for m=0:dt:tmax
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);

    F = -m2*(g*cos(theta) + l*dtheta^2)*sin(theta)...
        -kp*(m1 + m2*sin(theta)^2)*(m1*x - m2*l*sin(theta))...
        -kd*(m1*dx - m2*l*dtheta*cos(theta));

    dX1 = calculate_X(X, F, m1, m2, l, g);
    dX2 = calculate_X(X+0.5*dt*dX1, F, m1, m2, l, g);
    dX3 = calculate_X(X+0.5*dt*dX2, F, m1, m2, l, g);
    dX4 = calculate_X(X+dt*dX3, F, m1, m2, l, g);

    X = X + (dX1 + 2*dX2 + 2*dX3 + dX4)*dt/6;

    xout = [xout;X];
    fout = [fout;F];
end
%% 绘图
t = 0:dt:tmax;
figure;
subplot(2,2,1);
hold on;
grid on;
plot(t,xout(:,1),"b-","LineWidth",2);
xlabel("t(s)");ylabel("x(m)");
subplot(2,2,2);
hold on;
grid on;
plot(t,rad2deg(xout(:,3)),"b-","LineWidth",2);
xlabel("t(s)");ylabel("theta(reg)");
subplot(2,2,3);
% hold on;
% grid on;
% plot(t,xout(:,2),"b-","LineWidth",2);
% xlabel("t(s)");ylabel("v(m/s)");
% subplot(2,2,4);
% hold on;
% grid on;
% plot(t,rad2deg(xout(:,4)),"b-","LineWidth",2);
% xlabel("t(s)");ylabel("w(reg/s)");
% figure(2);
hold on;
grid on;
plot(t,fout,"b-","LineWidth",2);
xlabel("t(s)");ylabel("F");
%% 函数
function dX = calculate_X(X, F, m1, m2, l, g)
    x = X(1);
    dx = X(2);
    theta = X(3);
    dtheta = X(4);
    
    Delta = m1*m2*l + m2^2*l*sin(theta)^2;
    
    ddx = ( m2*l*F + m2^2*l^2*dtheta^2*sin(theta) + m2^2*l*g*sin(theta)*cos(theta) ) / Delta;
    
    ddtheta = ( -m2*cos(theta)*F - m2^2*l*dtheta^2*sin(theta)*cos(theta) ...
                - (m1 + m2)*m2*g*sin(theta) ) / Delta;
    
    dX = [dx, ddx, dtheta, ddtheta];
end
