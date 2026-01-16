clc;clear;close all;
m1 = 1;m2 = 1;l = 1;g = 9.8;
X = [1,deg2rad(35),0.2,deg2rad(2)];
dt = 0.01; t =0:0.01:50;
solution = [];
k1 = [10,10];k2 = [20,20];
for k=1:length(t)
    dX1 = cal_dX(X,g,m1,m2,l,k1,k2);
    dX2 = cal_dX(X + 0.5*dt*dX1,g,m1,m2,l,k1,k2);
    dX3 = cal_dX(X + 0.5*dt*dX2,g,m1,m2,l,k1,k2);
    dX4 = cal_dX(X + 0.5*dt*dX3,g,m1,m2,l,k1,k2);

    X = X + dt/6*(dX1 + 2*dX2 + 2*dX3 +dX4);
    solution = [solution;X];
end
figure;
hold on;grid on;
plot(t,solution(:,1),"LineWidth",2);
plot(t,solution(:,2),"LineWidth",2);
xlabel("t(s)");legend("x","theta");
function dX = cal_dX(X,g,m1,m2,l,k1,k2)
    % X = [x,theta,v,w];
    x = X(1);
    theta = X(2);
    v = X(3);
    w = X(4);
    kp1 = k1(1);kd1 = k1(1);
    kp2 = k2(1);kd2 = k2(1);
    u = -kp1*x - kd1*v;
    M = -kp2*theta - kd2*w;

    delta = m1 + m2*(sin(theta)^2);

    dv = (m2*g*cos(theta)*sin(theta) - m2*l*(w^2)*sin(theta) + u + cos(theta)*M/l)/delta;
    dw = (m1+m2)*g*sin(theta) -  m2*l*(w^2)*cos(theta)*sin(theta) + cos(theta)*u + (m1+m2)*M/m2/l;
    dw = dw/delta/l;
    dX = [v,w,dv,dw];
end