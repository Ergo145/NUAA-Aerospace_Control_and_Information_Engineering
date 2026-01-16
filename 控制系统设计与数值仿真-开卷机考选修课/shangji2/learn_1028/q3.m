clc;clear;

%% 参数设置
t_max   = 50;
J       = [1000,1500,1800];  
%X = [ftheta1 theta1 w1 ftheta2 theta2 w2 ftheta3 theta3 w3 ]
X       = [0,deg2rad(10),deg2rad(10),0,deg2rad(20),deg2rad(20),0,deg2rad(-30),deg2rad(-30)];
f       = [200,300,360];
dt      = 0.01;
t       = 0:dt:t_max;

%pid
kp1 = 6400;ki1 = 1400;kd1 = 5400;
kp2 = 6450;ki2 = 1450;kd2 = 5450;
kp3 = 6500;ki3 = 1500;kd3 = 5500;



Mout=[];
Xout=[];

%% 生成周期25秒、幅值30度的方波
T = 25; % 周期25秒
A = deg2rad(30);    %幅值三十度
% 生成方波
f_square = 1/T; % 频率 = 1/周期
y = A * square(2*pi*f_square*t);
y(1)=0;

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
    M1 = ki1*(0-ftheta1) + kp1*(y(i)-theta1) + kd1*(0-w1);
    M2 = ki2*(0-ftheta2) + kp2*(y(i)-theta2) + kd2*(0-w2);
    M3 = ki3*(0-ftheta3) + kp3*(y(i)-theta3) + kd3*(0-w3);

    M=[M1,M2,M3];
    Mout = [Mout;M];

    dX1 = individual(X,M,f,J);
    dX2 = individual(X+0.5*dt*dX1,M,f,J);
    dX3 = individual(X+0.5*dt*dX2,M,f,J);
    dX4 = individual(X+0.5*dt*dX3,M,f,J);

    X = X + (dX1 + 2*dX2 + 2*dX3 + dX4)*dt/6;
    Xout = [Xout;X];
end

%% 绘图
figure(1);
subplot(3,1,1)
hold on;
grid on;
plot(t, rad2deg(y),'r--', 'LineWidth', 1.5);
plot(t,rad2deg(Xout(:,2)),"b-",LineWidth=2);
legend("滚转角指令","滚转角响应");
subplot(3,1,2)
hold on;
grid on;
plot(t, rad2deg(y),'r--', 'LineWidth', 1.5);
plot(t,rad2deg(Xout(:,5)),"b-",LineWidth=2);
legend("偏航角指令","偏航角响应");
subplot(3,1,3)
hold on;
grid on;
plot(t, rad2deg(y),'r--', 'LineWidth', 1.5);
plot(t,rad2deg(Xout(:,8)),"b-",LineWidth=2);
legend("俯仰角指令","俯仰角响应");


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