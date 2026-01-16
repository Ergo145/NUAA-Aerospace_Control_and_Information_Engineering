clc;clear;
%% 仿真参数
% 初始值
m = 500;%20*500*9.8
g = 9.8;
% 气动参数
rho = 1.225;
S = 0.01;
C_Q = 0.1;
% 目标参数
vt = -20;
xt = 6000;
% 导弹参数
% X = [0,5000,500,0];
X = [0,5000,500,deg2rad(30)];  %老师程序
% 时间输出参数
k = 10;
dt = 0.01;
t = 0;
xout = [];
qout = [];
dqout = [];
Yout = [];
rout = [];
tout = [];
%% 仿真
[r,q,dq] = calculate_rqdq(X,xt,vt);
ym = X(2);
while r>0 && ym>0
    xm = X(1);
    ym = X(2);
    vm = X(3);
    thetam = X(4);
    
    [r,q,dq] = calculate_rqdq(X,xt,vt);
    [Q,Y] = calculate_QY(dq,vm,thetam,rho,k,m,g,S,C_Q);

    dX1 = calculate_dX(t,X,Q,Y,m,g);
    dX2 = calculate_dX(t,X+0.5*dt*dX1,Q,Y,m,g);
    dX3 = calculate_dX(t,X+0.5*dt*dX2,Q,Y,m,g);
    dX4 = calculate_dX(t,X+dt*dX3,Q,Y,m,g);

    X = X + (dX1 + 2*dX2 + 2*dX3 +dX4)*dt/6;

    xout = [xout;X];
    qout = [qout;q];
    dqout = [dqout;dq];
    Yout = [Yout;Y];
    rout = [rout;r];
    tout = [tout;t];

    t = t + dt;
    xt = xt + vt*dt;
end
%% 绘制
figure(1);
subplot(3,1,1);
hold on;grid on;
plot(tout,xout(:,3),"b-","LineWidth",2);
xlabel("t(s)");ylabel("vm(m/s)");
subplot(3,1,2);
hold on;grid on;
plot(tout,rad2deg(xout(:,4)),"b-","LineWidth",2);
xlabel("t(s)");ylabel("thetam(deg)");
subplot(3,1,3);
hold on;grid on;
plot(tout,Yout,"b-","LineWidth",2);
xlabel("t(s)");ylabel("u(N)");
figure(2);
subplot(3,1,1);
hold on;grid on;
plot(tout,rad2deg(qout),"b-","LineWidth",2);
xlabel("t(s)");ylabel("q(deg)");
subplot(3,1,2);
hold on;grid on;
plot(tout,rad2deg(dqout),"b-","LineWidth",2);
xlabel("t(s)");ylabel("dq(deg/s)");
subplot(3,1,3);
hold on;grid on;
plot(tout,rout,"b-","LineWidth",2);
xlabel("t(s)");ylabel("r(m)");
figure(3);
hold on;grid on;
plot(xout(:,1),xout(:,2),"r-","LineWidth",2);
plot(xout(end,1),xout(end,2),'*r','markersize',20),
set(gca,'fontname','microsoft yahei'),
xlabel("飞行纵程(m)");ylabel("飞行高度(m)");
title("导弹弹道和底面目标轨迹")
%% 函数
function dX = calculate_dX(t,X,Q,Y,m,g)
    xm = X(1);
    ym = X(2);
    vm = X(3);
    thetam = X(4);

    dxm = vm*cos(thetam);
    dym = vm*sin(thetam);
    dvm = -g*sin(thetam) - Q/m;
    dthetam = (-g*cos(thetam) + Y/m)/max(vm,1e-3);


    dX = [dxm,dym,dvm,dthetam];
end

function [Q,Y] = calculate_QY(dq,vm,thetam,rho,k,m,g,S,C_Q)
    Q = 0.5*rho*vm^2*S*C_Q;
    Y_temp = k*m*vm*dq + m*g*cos(thetam);
    if Y_temp > 20*m*g
        Y = 20*m*g;
    elseif  Y_temp < -20*m*g
        Y = -20*m*g;
    else 
        Y = Y_temp;
    end
end
function [r,q,dq] = calculate_rqdq(X,xt,vt)
    xm = X(1);
    ym = X(2);
    vm = X(3);
    thetam = X(4);

    r_temp = sqrt( (xm-xt)^2 + ym^2 );
    q_temp = -atan( ym/(xt-xm) );
    dq_temp = (vm*sin(q_temp - thetam) - vt*sin(q_temp))/r_temp;
    r = r_temp;
    q = q_temp;
    dq = dq_temp;
end