%% 一阶直线双倒立摆系统仿真
clc; clear; close all;

%% 参数设置
M = 1;        % 小车质量(kg)
m1 = 0.5;     % 第一杆质量(kg)
m2 = 0.5;     % 第二杆质量(kg)
L1 = 0.6;     % 第一杆长度(m)
L2 = 0.6;     % 第二杆长度(m)
g  = 9.8;     % 重力加速度(m/s^2)
F  = 5;       % 阶跃力(N)

%% 初始条件
x0   = 0;                 % 小车初始位置
dx0  = 0;                 % 小车初始速度
theta1_0 = deg2rad(180);  % 第一杆初始角度
dtheta1_0 = 0;            % 第一杆初始角速度
theta2_0 = deg2rad(180);  % 第二杆初始角度
dtheta2_0 = 0;            % 第二杆初始角速度
y0 = [x0; dx0; theta1_0; dtheta1_0; theta2_0; dtheta2_0];

%% 仿真时间
tspan = [0 10];

%% 求解微分方程
[t, y] = ode45(@(t, y) pendulum_dynamics(t, y, M, m1, m2, L1, L2, g, F), tspan, y0);

%% 计算角加速度与加速度
for i = 1:length(t)
    dydt = pendulum_dynamics(t(i), y(i,:)', M, m1, m2, L1, L2, g, F);
    ddx(i,1) = dydt(2);
    ddtheta1(i,1) = dydt(4);
    ddtheta2(i,1) = dydt(6);
end

%% 提取数据
x = y(:,1); dx = y(:,2);
theta1 = y(:,3); dtheta1 = y(:,4);
theta2 = y(:,5); dtheta2 = y(:,6);

%% 绘图
figure('Position',[100 100 1000 800])

subplot(3,2,1)
plot(t, rad2deg(theta1)); xlabel('t/s'); ylabel('\theta_1 (deg)'); grid on
title('角度 θ_1')

subplot(3,2,2)
plot(t, rad2deg(theta2)); xlabel('t/s'); ylabel('\theta_2 (deg)'); grid on
title('角度 θ_2')

subplot(3,2,3)
plot(t, dtheta1); xlabel('t/s'); ylabel('\omega_1 (rad/s)'); grid on
title('角速度 θ̇₁')

subplot(3,2,4)
plot(t, dtheta2); xlabel('t/s'); ylabel('\omega_2 (rad/s)'); grid on
title('角速度 θ̇₂')

subplot(3,2,5)
plot(t, ddtheta1); xlabel('t/s'); ylabel('\alpha_1 (rad/s²)'); grid on
title('角加速度 θ̈₁')

subplot(3,2,6)
plot(t, ddtheta2); xlabel('t/s'); ylabel('\alpha_2 (rad/s²)'); grid on
title('角加速度 θ̈₂')

figure
subplot(3,1,1)
plot(t, x); xlabel('t/s'); ylabel('x/m'); grid on; title('位置 x')
subplot(3,1,2)
plot(t, dx); xlabel('t/s'); ylabel('v/m·s⁻¹'); grid on; title('速度 ẋ')
subplot(3,1,3)
plot(t, ddx); xlabel('t/s'); ylabel('a/m·s⁻²'); grid on; title('加速度 ẍ')

%% 动力学方程函数
function dydt = pendulum_dynamics(~, y, M, m1, m2, L1, L2, g, F)
x = y(1); dx = y(2);
theta1 = y(3); dtheta1 = y(4);
theta2 = y(5); dtheta2 = y(6);

% 根据题目中的简化模型
ddx = F*(4/(4*M + m1 + m2)) ...
      - (3*m1*g*sin(theta1))/(4*M + m1 + m2) ...
      - (3*m2*g*sin(theta2))/(4*M + m1 + m2);

ddtheta1 = (3*g*(4*M + 4*m1 + m2)*sin(theta1))/(4*L1*(4*M + m1 + m2)) ...
           + (9*m2*g*sin(theta2))/(4*L1*(4*M + m1 + m2)) ...
           - F*3/(L1*(4*M + m1 + m2));

ddtheta2 = (3*g*(4*M + 4*m1 + m2)*sin(theta2))/(4*L2*(4*M + m1 + m2)) ...
           + (9*m1*g*sin(theta1))/(4*L2*(4*M + m1 + m2)) ...
           - F*3/(L2*(4*M + m1 + m2));

dydt = [dx; ddx; dtheta1; ddtheta1; dtheta2; ddtheta2];
end
