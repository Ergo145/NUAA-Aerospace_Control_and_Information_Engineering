close all, clc, clear
global j1 j2 j3
j1 = 1000;
j2 = 1500;
j3 = 1800;

% 初始状态
yita1 = 0; yita2 = 0; yita3 = 0;
theta1 = deg2rad(10); theta2 = deg2rad(20); theta3 = deg2rad(30);
w1 = deg2rad(10); w2 = deg2rad(20); w3 = deg2rad(-30);

state = [0; 0; 0; theta1; theta2; theta3; w1; w2; w3];
stateout = state;

% 期望角度（设定点）
theta1_desired = deg2rad(0);
theta2_desired = deg2rad(0);
theta3_desired = deg2rad(0);

% 时间设置
t0 = 0; tf = 20; dt = 0.01;
tout = t0:dt:tf;

% 初始化输出数组
M1out = []; M2out = []; M3out = [];

% 修正的PID参数 - 需要负反馈
kp1 = 1000; kp2 = 1500; kp3 = 1800;    % 比例项
ki1 = 100;  ki2 = 100;  ki3 = 100;     % 积分项  
kd1 = 1000; kd2 = 1500; kd3 = 1800;    % 微分项

% 扰动
f1 = 0; f2 = 0; f3 = 0;

% 误差积分初始化
int_e1 = 0; int_e2 = 0; int_e3 = 0;

for i = 1:length(tout)
    t = tout(i);
    
    % 当前状态
    theta1 = state(4); theta2 = state(5); theta3 = state(6);
    w1 = state(7); w2 = state(8); w3 = state(9);
    
    % 计算误差
    e1 = theta1_desired - theta1;
    e2 = theta2_desired - theta2; 
    e3 = theta3_desired - theta3;
    
    % 积分误差
    int_e1 = int_e1 + e1 * dt;
    int_e2 = int_e2 + e2 * dt;
    int_e3 = int_e3 + e3 * dt;
    
    % PID控制
    M1 = kp1 * e1 + ki1 * int_e1 + kd1 * (-w1);  % 角速度与误差方向相反
    M2 = kp2 * e2 + ki2 * int_e2 + kd2 * (-w2);
    M3 = kp3 * e3 + ki3 * int_e3 + kd3 * (-w3);
    
    M1out = [M1out, M1];
    M2out = [M2out, M2]; 
    M3out = [M3out, M3];
   
    % RK4积分
    ke1 = attitude_dynamics(t, state, M1, M2, M3, f1, f2, f3);
    ke2 = attitude_dynamics(t+0.5*dt, state+0.5*ke1*dt, M1, M2, M3, f1, f2, f3);
    ke3 = attitude_dynamics(t+0.5*dt, state+0.5*ke2*dt, M1, M2, M3, f1, f2, f3);
    ke4 = attitude_dynamics(t+dt, state+ke3*dt, M1, M2, M3, f1, f2, f3);

    state = state + (ke1 + 2*ke2 + 2*ke3 + ke4) * dt / 6;
    stateout = [stateout, state];
end

% 处理输出
tout = [tout, tout(end)+dt];
M1out = [M1out, M1out(end)];
M2out = [M2out, M2out(end)]; 
M3out = [M3out, M3out(end)];

theta1out = stateout(4,:);
theta2out = stateout(5,:);
theta3out = stateout(6,:);

% 绘图
figure;
subplot(2,1,1);
plot(tout, rad2deg(theta1out), 'r', 'LineWidth', 2);
hold on;
plot(tout, rad2deg(theta2out), 'g', 'LineWidth', 2);
plot(tout, rad2deg(theta3out), 'b', 'LineWidth', 2);
hold off;
ylabel('角度(deg)');
legend('θ₁', 'θ₂', 'θ₃');
grid on;

subplot(2,1,2);
plot(tout, M1out, 'r', 'LineWidth', 2);
hold on;
plot(tout, M2out, 'g', 'LineWidth', 2);
plot(tout, M3out, 'b', 'LineWidth', 2);
hold off;
ylabel('M(N.M)');
legend(' $M_1$ ', '$M_2$', '$M_3$','Interpreter','latex');
grid on;
