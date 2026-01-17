clear; clc; close all;

%% 1. 参数设置
global mu
mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
Re = 6371;              % 地球半径 (km)

% 初始状态
r0 = [-5102; -8228; -2105]; % km
v0 = [-4.348; 3.478; -2.846]; % km/s
y0 = [r0; v0];

% 仿真时间设置
h = 10;                 % 积分步长 10秒
t_span = [0, 15000];    % 仿真时长 (秒)

%% 2. 数值积分求解
[t_eu, y_eu]   = solve_ode_custom(@sys_model, t_span, y0, h, 'euler');
[t_imp, y_imp] = solve_ode_custom(@sys_model, t_span, y0, h, 'improved_euler');
[t_rk4, y_rk4] = solve_ode_custom(@sys_model, t_span, y0, h, 'rk4');

% 计算 r 和 v 的模长
r_norm_eu = sqrt(sum(y_eu(:,1:3).^2, 2)); v_norm_eu = sqrt(sum(y_eu(:,4:6).^2, 2));
r_norm_imp = sqrt(sum(y_imp(:,1:3).^2, 2)); v_norm_imp = sqrt(sum(y_imp(:,4:6).^2, 2));
r_norm_rk4 = sqrt(sum(y_rk4(:,1:3).^2, 2)); v_norm_rk4 = sqrt(sum(y_rk4(:,4:6).^2, 2));


%% 3. 绘图结果

% --- 图窗 1: 三维轨迹对比 (三种方法叠加) ---
figure(1); set(gcf, 'Color', 'w', 'Name', 'Figure 1: 3D Trajectory Comparison');
plot3(y_eu(:,1), y_eu(:,2), y_eu(:,3), 'g--', 'LineWidth', 2); hold on;
plot3(y_imp(:,1), y_imp(:,2), y_imp(:,3), 'b-', 'LineWidth', 2);
plot3(y_rk4(:,1), y_rk4(:,2), y_rk4(:,3), 'r-', 'LineWidth', 2.5);

% 画地球示意及图例
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
plot3(0, 0, 0, 'Color', 'blue', 'LineWidth', 10, 'HandleVisibility', 'on', 'DisplayName', '地球');

grid on; axis equal; view(3);
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
legend('欧拉法', '改进欧拉法', 'RK4', '地球');
title('任务1: 三种数值积分方法的轨道对比 (h=10s)');


% --- 图窗 2, 3, 4: 状态分量分解图 (按方法分离) ---
% Figure 2: 欧拉法状态分量
plot_state_components(t_eu, y_eu, '欧拉法', 2, 'b-');
% Figure 3: 改进欧拉法状态分量
plot_state_components(t_imp, y_imp, '改进欧拉法', 3, 'b-');
% Figure 4: RK4 法状态分量
plot_state_components(t_rk4, y_rk4, 'RK4 法', 4, 'b-');


% --- 图窗 5, 6, 7: 模长分解图 (按方法分离) ---
% Figure 5: 欧拉法模长
plot_magnitude_components(t_eu, r_norm_eu, v_norm_eu, '欧拉法', 5, 'b-');
% Figure 6: 改进欧拉法模长
plot_magnitude_components(t_imp, r_norm_imp, v_norm_imp, '改进欧拉法', 6, 'b-');
% Figure 7: RK4 法模长
plot_magnitude_components(t_rk4, r_norm_rk4, v_norm_rk4, 'RK4 法', 7, 'b-');


%% --- 局部函数定义 ---

% 二体动力学模型
function dydt = sys_model(~, y)
    global mu
    r_vec = y(1:3);
    v_vec = y(4:6);
    r = norm(r_vec);
    a_vec = -mu * r_vec / r^3;
    dydt = [v_vec; a_vec];
end

% 通用求解器框架
function [t, y] = solve_ode_custom(fun, t_span, y0, h, method)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    for i = 1:length(t)-1
        curr_t = t(i);
        curr_y = y(i, :)';
        switch method
            case 'euler'
                k1 = fun(curr_t, curr_y);
                y(i+1, :) = curr_y + h * k1;
            case 'improved_euler'
                k1 = fun(curr_t, curr_y);
                y_pred = curr_y + h * k1;
                k2 = fun(curr_t + h, y_pred);
                y(i+1, :) = curr_y + (h/2) * (k1 + k2);
            case 'rk4'
                k1 = fun(curr_t, curr_y);
                k2 = fun(curr_t + 0.5*h, curr_y + 0.5*h*k1);
                k3 = fun(curr_t + 0.5*h, curr_y + 0.5*h*k2);
                k4 = fun(curr_t + h, curr_y + h*k3);
                y(i+1, :) = y(i, :)' + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
        end
    end
end

% 绘制六个状态分量图的辅助函数 (x,y,z 左侧; vx,vy,vz 右侧)
function plot_state_components(t, y, method_name, fig_num, line_style)
    figure(fig_num); 
    set(gcf, 'Color', 'w', 'Name', ['Figure ' num2str(fig_num) ': ' method_name ' State Components']);
    
    titles_r = {'X', 'Y', 'Z'};
    titles_v = {'V_X', 'V_Y', 'V_Z'};
    units = {'km', 'km', 'km', 'km/s', 'km/s', 'km/s'};

    for i = 1:3 % 循环 3 次，代表三行
        % 左侧列 (Position: x, y, z) -> subplots 1, 3, 5
        subplot(3, 2, 2*i - 1);
        plot(t, y(:, i), line_style, 'LineWidth', 2);
        ylabel(sprintf('%s (%s)', titles_r{i}, units{i}));
        title(sprintf('%s 随时间变化', titles_r{i}));
        grid on;
        if i == 3, xlabel('时间 t (s)'); end
        
        % 右侧列 (Velocity: vx, vy, vz) -> subplots 2, 4, 6
        subplot(3, 2, 2*i);
        plot(t, y(:, i+3), line_style, 'LineWidth', 2);
        ylabel(sprintf('%s (%s)', titles_v{i}, units{i+3}));
        title(sprintf('%s 随时间变化', titles_v{i}));
        grid on;
        if i == 3, xlabel('时间 t (s)'); end
    end
    sgtitle([method_name ': 状态分量随时间变化']); % 总标题
end

% 绘制 r 和 v 模长图的辅助函数 (按方法分离)
function plot_magnitude_components(t, r_norm, v_norm, method_name, fig_num, line_style)
    figure(fig_num); 
    set(gcf, 'Color', 'w', 'Name', ['Figure ' num2str(fig_num) ': ' method_name ' Magnitude Comparison']);

    % 子图 1: 向径 r 模长
    subplot(2, 1, 1);
    plot(t, r_norm, line_style, 'LineWidth', 2); 
    xlabel('时间 t (s)'); ylabel('向径模长 r (km)');
    title('向径模长 r 随时间变化');
    grid on;

    % 子图 2: 速度 v 模长
    subplot(2, 1, 2);
    plot(t, v_norm, line_style, 'LineWidth', 2);
    xlabel('时间 t (s)'); ylabel('速度模长 v (km/s)');
    title('速度模长 v 随时间变化');
    grid on;
    
    sgtitle([method_name ': 模长随时间变化']); % 总标题
end