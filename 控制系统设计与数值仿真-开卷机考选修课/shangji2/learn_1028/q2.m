clc;clear;

%% 参数设置
t_max   = 20;
% J1      = 1000;
% J2      = 1500;
% J3      = 1800;
J       = [1000,1500,1800];  
%X = [ftheta1 theta1 w1 ftheta2 theta2 w2 ftheta3 theta3 w3 ]
X       = [0,deg2rad(10),deg2rad(10),0,deg2rad(20),deg2rad(20),0,deg2rad(-30),deg2rad(-30)];
% f1      = 200;
% f2      = 300;
% f3      = 360;
f       = [200,300,360];
dt      = 0.01;
t       = 0:dt:t_max;
% 
% %pid
% kp1 = 4000;ki1 = 1100;kd1 = 1900;
% kp2 = 4500;ki2 = 2500;kd2 = 2400;
% kp3 = 5000;ki3 = 2800;kd3 = 2700;
kp1 = 1800;ki1 = 200;kd1 = 1500;
kp2 = 1800;ki2 = 600;kd2 = 1500;
kp3 = 1800;ki3 = 400;kd3 = 1500;


Mout=[];
Xout=[];
%% 仿真
for i=1:length(t)
    ftheta1 = X(1);
    theta1  = X(2);
    w1      = X(3);
    ftheta2 = X(4);
    theta2  = X(5);
    w2      = X(6);
    ftheta3 = X(7);
    theta3  = X(8);
    w3      = X(9);
    M1 = ki1*(0-ftheta1) + kp1*(0-theta1) + kd1*(0-w1);
    M2 = ki2*(0-ftheta2) + kp2*(0-theta2) + kd2*(0-w2);
    M3 = ki3*(0-ftheta3) + kp3*(0-theta3) + kd3*(0-w3);

    M=[M1,M2,M3];
    Mout = [Mout;M];

    dX1 = individual(X,M,f,J);
    dX2 = individual(X+0.5*dt*dX1,M,f,J);
    dX3 = individual(X+0.5*dt*dX2,M,f,J);
    dX4 = individual(X+0.5*dt*dX3,M,f,J);

    X = X + (dX1 + 2*dX2 + 2*dX3 + dX4)*dt/6;
    Xout = [Xout;X];
end

figure;
subplot(2,1,1);
hold on;
grid on;
plot(t,rad2deg(Xout(:,2)),"-",LineWidth=2);
plot(t,rad2deg(Xout(:,5)),"-",LineWidth=2);
plot(t,rad2deg(Xout(:,8)),"-",LineWidth=2);
legend("\theta_1","\theta_2","\theta_3");
subplot(2,1,2);
hold on;
grid on;
plot(t,Mout(:,1),"-",LineWidth=2);
plot(t,Mout(:,2),"-",LineWidth=2);
plot(t,Mout(:,3),"-",LineWidth=2);
legend("M_1","M_2","M_3");

%% 计算函数
function dX = individual(X,M,f,J)
    ftheta1 = X(1);
    theta1  = X(2);
    w1      = X(3);
    ftheta2 = X(4);
    theta2  = X(5);
    w2      = X(6);
    ftheta3 = X(7);
    theta3  = X(8);
    w3      = X(9);

    J1 = J(1);J2 = J(2);J3 = J(3);
    f1 = f(1);f2 = f(2);f3 = f(3);
    M1 = M(1);M2 = M(2);M3 = M(3);

    dtheta1  = w1 - w2*cos(theta1)*tan(theta3) + w3*sin(theta1)*tan(theta3);
    dtheta2  = w2*cos(theta1)/cos(theta3) - w3*sin(theta1)/cos(theta3);
    dtheta3  = w2*sin(theta1) + w3*cos(theta1);

    dw1 = (J2 - J3)*w2*w3/J1 + (M1+f1)/J1;
    dw2 = (J3 - J1)*w3*w1/J2 + (M2+f2)/J2;
    dw3 = (J1 - J2)*w1*w2/J3 + (M3+f3)/J3;

    dX = [theta1,dtheta1,dw1,theta2,dtheta2,dw2,theta3,dtheta3,dw3];
end