clc;clear;close all;
m = 500;g = 9.8;rho = 1.225;S = 0.01;C_Q=0.1;k = 10;
xt = 6000;vt = -20;
t = 0; dt = 0.01;
X = [0,5000,500,deg2rad(30)];
solution = [];

[r,q,dq] = cal_qdqr(X,xt,vt);
while r>1e-1 && X(2)>0
    [r,q,dq] = cal_qdqr(X,xt,vt);
    [Q,Y] = cal_QY(X,dq,rho,S,C_Q,k,m,g);
    
    % 存储当前状态
    solution = [solution;t,X,r,q,dq,Q,Y];
    
    % 状态更新
    dX1 = cal_dX(X,Q,Y,g,m);
    dX2 = cal_dX(X + 0.5*dt*dX1,Q,Y,g,m);
    dX3 = cal_dX(X + 0.5*dt*dX2,Q,Y,g,m);
    dX4 = cal_dX(X + 0.5*dt*dX3,Q,Y,g,m);
    X = X + dt*(dX1 + 2*dX2 + 2*dX3 + dX4)/6;
    t = t + dt;
    xt = xt + vt*dt;

end
figure(1);
subplot(3,1,1);
hold on;grid on;
plot(solution(:,1),solution(:,4),"LineWidth",2);
xlabel("t(s)");ylabel("vm(m/s)");
subplot(3,1,2);
hold on;grid on;
plot(solution(:,1),rad2deg(solution(:,5)),"LineWidth",2);
xlabel("t(s)");ylabel("\thetam (deg)");
subplot(3,1,3);
hold on;grid on;
plot(solution(:,1),solution(:,10),"LineWidth",2);
xlabel("t(s)");ylabel("vm(m/s)");
figure(2);
subplot(3,1,1);
hold on;grid on;
plot(solution(:,1),rad2deg(solution(:,7)),"LineWidth",2);
xlabel("t(s)");ylabel("q(deg)");
subplot(3,1,2);
hold on;grid on;
plot(solution(:,1),rad2deg(solution(:,8)),"LineWidth",2);
xlabel("t(s)");ylabel("\dot{q} (deg/s)");
subplot(3,1,3);
hold on;grid on;
plot(solution(:,1),solution(:,6),"LineWidth",2);
xlabel("t(s)");ylabel("r(m/s)");
figure(3);
hold on;grid on;
plot(solution(:,2),solution(:,3),"LineWidth",2);
xlabel("飞行纵程(m)");ylabel("飞行高度(m)");
title("导弹弹道和地面目标轨迹");

function [r,q,dq] = cal_qdqr(X,xt,vt)
    xm     =X(1);
    ym     =X(2);
    vm     =X(3);
    thetam =X(4);
    r_temp = sqrt( (xm - xt)^2 + ym^2);
    q_temp = -atan(ym/(xt - xm));
    dq = (vm*sin(q_temp - thetam) - vt*sin(q_temp))/r_temp;
    r = r_temp;
    q = q_temp; 
end

function [Q,Y] = cal_QY(X,dq,rho,S,C_Q,k,m,g)
    xm     =X(1);
    ym     =X(2);
    vm     =X(3);
    thetam =X(4);
    Q = 0.5*rho*vm^2*S*C_Q;
    Y_temp = k*m*vm*dq + m*g*cos(thetam);
    if Y_temp>20*m*g
        Y = 20*m*g;
    elseif  Y_temp<-20*m*g
        Y = -20*m*g;
    else
        Y = Y_temp;
    end
end

function dX = cal_dX(X,Q,Y,g,m)
    xm     =X(1);
    ym     =X(2);
    vm     =X(3);
    thetam =X(4);

    dxm = vm*cos(thetam);
    dym = vm*sin(thetam);
    dvm = -g*sin(thetam) - Q/m;
    dthetam = (-g*cos(thetam) + Y/m)/vm;

    dX = [dxm,dym,dvm,dthetam];
end