%% 复刻图 13.4 (科氏力) & 13.5 (离心力) - 粒子群仿真
clc; clear; close all;

% --- 初始化粒子群 ---
num_particles = 12; % 书中图示大约有12个点
theta = linspace(0, 2*pi, num_particles+1); 
theta(end) = []; % 去掉重复点
radius = 4;      % 初始分布半径

% 初始位置：分布在圆周上
X0_all = radius * cos(theta);
Y0_all = radius * sin(theta);

figure('Name', '复刻图13.4与13.5：力场分解演示', 'Color', 'w', 'Position', [100, 100, 1000, 450]);

% === 子图1：复刻图 13.4 (仅受科里奥利力) ===
% 特征：F_cor = -2*m*(omega x v)。仅改变方向，轨迹为圆圈。
subplot(1, 2, 1); hold on;
title('图 13.4 仅受科里奥利力 (Coriolis Force Only)');
axis equal; grid on; box on;
xlim([-15 15]); ylim([-15 15]);

% 模拟每个粒子向圆心运动的初速度
v_in = 3; 
for i = 1:num_particles
    % 初始状态：位置在圆周，速度指向圆心
    state0 = [X0_all(i); Y0_all(i); -v_in*cos(theta(i)); -v_in*sin(theta(i))];
    [~, Y_cor] = ode45(@(t,y) [y(3); y(4); 2*y(4); -2*y(3)], [0, 3], state0);
    
    % 绘图
    h_traj = plot(Y_cor(:,1), Y_cor(:,2), 'k-', 'LineWidth', 1.2); % 轨迹
    h_start = plot(X0_all(i), Y0_all(i), 'k.', 'MarkerSize', 15);  % 起点
    
    % 绘制箭头 (表示运动方向)
    quiver(Y_cor(end,1), Y_cor(end,2), Y_cor(end,3), Y_cor(end,4), ...
        0.5, 'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);
end
legend([h_start, h_traj], {'Start Point (起点)', 'Trajectory (轨迹)'}, 'Location', 'best');
xlabel('x'); ylabel('y');

% === 子图2：复刻图 13.5 (仅受离心力) ===
% 特征：F_cen = m*omega^2*r。沿径向向外加速，轨迹为指数发散直线。
subplot(1, 2, 2); hold on;
title('图 13.5 仅受离心力 (Centrifugal Force Only)');
axis equal; grid on; box on;
xlim([-15 15]); ylim([-15 15]);

for i = 1:num_particles
    % 初始状态：位置在圆周，初速度为0 (书中示例似乎初速度极小或为0)
    state0 = [X0_all(i); Y0_all(i); 0; 0]; 
    [~, Y_cen] = ode45(@(t,y) [y(3); y(4); y(1); y(2)], [0, 1.2], state0);
    
    % 绘图
    plot(Y_cen(:,1), Y_cen(:,2), 'k-', 'LineWidth', 1.2);
    plot(X0_all(i), Y0_all(i), 'k.', 'MarkerSize', 15);
    
    % 绘制向外的箭头
    quiver(Y_cen(end,1), Y_cen(end,2), Y_cen(end,1), Y_cen(end,2), ...
        0.2, 'k', 'LineWidth', 1, 'MaxHeadSize', 0.5);
end
legend('Particle Motion (粒子运动)', 'Location', 'best');
xlabel('x'); ylabel('y');