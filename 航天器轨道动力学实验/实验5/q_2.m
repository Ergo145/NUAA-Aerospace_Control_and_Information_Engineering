clear; clc; close all;

global mu
mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
Re = 6371;              % 地球半径 (km)

% 初始状态
r0 = [-5102; -8228; -2105];
v0 = [-4.348; 3.478; -2.846];
y0 = [r0; v0];

% 定义不同的步长 (秒)
step_sizes = [10, 100, 300, 600]; 
t_end = 25000; % 仿真时长 (稍微加长以观察更明显的发散)

fprintf('开始仿真...\n');
fprintf('比较方法: 欧拉法, 改进欧拉法, 4阶龙格库塔法\n');
fprintf('步长序列: [%s] 秒\n', num2str(step_sizes));

%% --- 第一部分：欧拉法 (Euler Method) ---
fprintf('正在计算: 欧拉法...\n');
figure(1); set(gcf, 'Color', 'w', 'Name', 'Fig 1: 欧拉法 - 3D轨迹');
sgtitle('欧拉法：不同积分步长下的轨迹发散情况');
figure(2); set(gcf, 'Color', 'w', 'Name', 'Fig 2: 欧拉法 - 能量守恒');
sgtitle('欧拉法：比机械能随时间变化');

plot_method_results(@solve_euler, step_sizes, y0, t_end, Re, mu, 1, 2);

%% --- 第二部分：改进欧拉法 (Improved Euler Method) ---
fprintf('正在计算: 改进欧拉法...\n');
figure(3); set(gcf, 'Color', 'w', 'Name', 'Fig 3: 改进欧拉法 - 3D轨迹');
sgtitle('改进欧拉法：不同积分步长下的轨迹发散情况');
figure(4); set(gcf, 'Color', 'w', 'Name', 'Fig 4: 改进欧拉法 - 能量守恒');
sgtitle('改进欧拉法：比机械能随时间变化');

plot_method_results(@solve_improved_euler, step_sizes, y0, t_end, Re, mu, 3, 4);

%% --- 第三部分：四阶龙格库塔法 (RK4 Method) ---
fprintf('正在计算: RK4...\n');
figure(5); set(gcf, 'Color', 'w', 'Name', 'Fig 5: RK4 - 3D轨迹');
sgtitle('RK4算法：不同积分步长下的轨迹发散情况');
figure(6); set(gcf, 'Color', 'w', 'Name', 'Fig 6: RK4 - 能量守恒');
sgtitle('RK4算法：比机械能随时间变化');

plot_method_results(@solve_rk4, step_sizes, y0, t_end, Re, mu, 5, 6);

fprintf('仿真完成！请查看生成的6个图窗。\n');

%% ================= 辅助绘图函数 =================
function plot_method_results(solver_func, step_sizes, y0, t_end, Re, mu, fig_traj, fig_energy)
    % 该函数负责循环不同的步长，调用指定的求解器，并绘图
    for i = 1:length(step_sizes)
        h = step_sizes(i);
        
        % 调用传入的求解器函数
        [t, y] = solver_func(@sys_model, [0, t_end], y0, h);
        
        % --- 绘制 3D 轨迹 ---
        figure(fig_traj);
        subplot(2, 2, i);
        hold on;
        % 画地球
        [X_e,Y_e,Z_e] = sphere(20); 
        surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0.8 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
        % 画轨迹
        plot3(y(:,1), y(:,2), y(:,3), 'b-', 'LineWidth', 1.2);
        % 起点标记
        plot3(y(1,1), y(1,2), y(1,3), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 4);
        
        axis equal; grid on; view(3);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        title(sprintf('步长 h = %d s', h));
        
        % --- 计算并绘制能量 ---
        r_mag = sqrt(sum(y(:,1:3).^2, 2));
        v_sq = sum(y(:,4:6).^2, 2);
        Energy = v_sq/2 - mu./r_mag;
        
        figure(fig_energy);
        subplot(2, 2, i);
        plot(t, Energy, 'r-', 'LineWidth', 1.5);
        
        xlabel('时间 (s)'); ylabel('比机械能');
        title(sprintf('步长 h = %d s', h));
        grid on; 
        
        % 自动调整Y轴范围以突出发散（如果能量变化很大）
        if max(Energy) - min(Energy) > 100
             % 如果发散严重，保持默认试图
        else
             % 如果相对稳定，稍微放大一点看细节
             ylim([min(Energy)-0.1, max(Energy)+0.1]);
        end
    end
end

%% ================= 动力学模型与数值积分器 =================

% 1. 二体问题动力学模型
function dydt = sys_model(~, y)
    global mu
    r_vec = y(1:3);
    v_vec = y(4:6);
    r = norm(r_vec);
    dydt = [v_vec; -mu * r_vec / r^3];
end

% 2. 欧拉法 (Euler)
function [t, y] = solve_euler(fun, t_span, y0, h)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    for i = 1:length(t)-1
        dydt = fun(t(i), y(i, :)');
        y(i+1, :) = y(i, :) + (h * dydt)';
    end
end

% 3. 改进欧拉法 (Improved Euler / Heun's Method)
function [t, y] = solve_improved_euler(fun, t_span, y0, h)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    for i = 1:length(t)-1
        curr_y = y(i, :)';
        
        % 预测步 (Predictor)
        k1 = fun(t(i), curr_y);
        y_predict = curr_y + h * k1;
        
        % 校正步 (Corrector)
        k2 = fun(t(i) + h, y_predict);
        y(i+1, :) = curr_y' + (h/2 * (k1 + k2))';
    end
end

% 4. 四阶龙格库塔法 (RK4)
function [t, y] = solve_rk4(fun, t_span, y0, h)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    for i = 1:length(t)-1
        curr_t = t(i);
        curr_y = y(i, :)';
        
        k1 = fun(curr_t, curr_y);
        k2 = fun(curr_t + h/2, curr_y + h*k1/2);
        k3 = fun(curr_t + h/2, curr_y + h*k2/2);
        k4 = fun(curr_t + h, curr_y + h*k3);
        
        y(i+1, :) = curr_y' + (h/6 * (k1 + 2*k2 + 2*k3 + k4))';
    end
end