clear; clc; close all;

global mu
mu = 398600.4418;
Re = 6371;

% 初始状态
r0 = [-5102; -8228; -2105];
v0 = [-4.348; 3.478; -2.846];
y0 = [r0; v0];

% 使用高精度RK4求解，步长要小以保证曲线平滑
h = 10;
t_span = [0, 18000]; % 约5小时
[t, y] = solve_rk4(@sys_model, t_span, y0, h);

%% 绘图
figure(1); set(gcf, 'Color', 'w');
% 1. 画完整轨道线
plot3(y(:,1), y(:,2), y(:,3), 'k-', 'LineWidth', 1); hold on;

% 2. 画地球
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.2);

% 3. 叠加等时间间隔点
% 设定打点的时间间隔，例如每 600 秒画一个点
time_interval = 600; 
step_interval = time_interval / h; % 计算对应的数组索引间隔

scatter3(y(1:step_interval:end, 1), ...
         y(1:step_interval:end, 2), ...
         y(1:step_interval:end, 3), ...
         40, 'r', 'filled');

axis equal; grid on; view(3);
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title({'任务3: 轨道等时间间隔点分布', '点稀疏=速度快(近地点), 点密集=速度慢(远地点)'});

%% --- 局部函数 ---
function dydt = sys_model(~, y)
    global mu
    r_vec = y(1:3);
    v_vec = y(4:6);
    r = norm(r_vec);
    dydt = [v_vec; -mu * r_vec / r^3];
end

function [t, y] = solve_rk4(fun, t_span, y0, h)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    for i = 1:length(t)-1
        k1 = fun(t(i), y(i, :)');
        k2 = fun(t(i) + 0.5*h, y(i, :)' + 0.5*h*k1);
        k3 = fun(t(i) + 0.5*h, y(i, :)' + 0.5*h*k2);
        k4 = fun(t(i) + h, y(i, :)' + h*k3);
        y(i+1, :) = y(i, :)' + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
end