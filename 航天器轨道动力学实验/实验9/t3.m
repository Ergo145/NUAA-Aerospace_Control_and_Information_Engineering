%% 实验内容3：CRTBP 轨道仿真 (含碰撞检测修正版)
clc; clear; close all;

% --- 全局参数 ---
global mu
mu = 0.1; % 按照图12.5的设置
C_target = 3.4; % 目标雅可比常数

% --- 初始条件组 ---
x0_list = [-0.2, 0, 0.2, 0.7]; 
t_span = [0, 15]; % 积分时间

figure('Color', 'w', 'Position', [100, 100, 800, 700]);

for i = 1:length(x0_list)
    x0 = x0_list(i);
    y0 = 0;   
    vx0 = 0;  
    
    % --- 1. 计算初始速度 vy0 ---
    r1_0 = sqrt((x0 + mu)^2 + y0^2);
    r2_0 = sqrt((x0 - 1 + mu)^2 + y0^2);
    Omega_0 = 0.5 * (x0^2 + y0^2) + (1 - mu) / r1_0 + mu / r2_0;
    
    v_sq = 2 * Omega_0 - C_target;
    if v_sq < 0
        fprintf('Case %d: x0=%.2f 能量不足，无法起飞。\n', i, x0);
        continue;
    end
    vy0 = sqrt(v_sq); 
    
    % --- 2. 积分求解 (添加了 Events 选项) ---
    % 设置 RelTol/AbsTol 精度，并挂载 detectCrash 事件函数
    options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @detectCrash);
    
    state0 = [x0; y0; vx0; vy0];
    
    % 使用 try-catch 结构防止极其罕见的积分器错误中断循环
    try
        [t, state, te, ye, ie] = ode45(@crtbp_ode, t_span, state0, options);
    catch ME
        warning(['积分在 x0=', num2str(x0), ' 处失败: ', ME.message]);
        continue;
    end
    
    % --- 3. 绘图 ---
    subplot(2, 2, i);
    hold on;
    
    % A. 绘制背景：运动禁止区域
    [X, Y] = meshgrid(-1.6:0.02:1.6, -1.6:0.02:1.6);
    R1 = sqrt((X + mu).^2 + Y.^2);
    R2 = sqrt((X - 1 + mu).^2 + Y.^2);
    W = 0.5 * (X.^2 + Y.^2) + (1 - mu) ./ R1 + mu ./ R2;
    contour(X, Y, 2*W, [C_target, C_target], 'k', 'LineWidth', 1); 
    
    % B. 绘制轨迹
    plot(state(:,1), state(:,2), 'b', 'LineWidth', 1.2);
    
    % 如果发生碰撞，标记碰撞点
    if ~isempty(te)
        plot(ye(:,1), ye(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        text(ye(:,1), ye(:,2)+0.1, 'Crash', 'Color', 'r', 'FontSize', 8);
    end
    
    % C. 绘制天体和起点
    plot(-mu, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 6); % P1
    plot(1-mu, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4); % P2
    plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5); % 起点
    
    title(['x_0 = ', num2str(x0)]);
    xlabel('x'); ylabel('y');
    axis equal;
    xlim([-1.6, 1.6]); ylim([-1.6, 1.6]);
    grid on;
end

sgtitle(['轨道仿真: C = ', num2str(C_target), ', \mu = ', num2str(mu)]);


% -------------------------------------------------------------------------
% 动力学方程函数
% -------------------------------------------------------------------------
function dstate = crtbp_ode(~, state)
    global mu
    x = state(1); y = state(2); vx = state(3); vy = state(4);
    
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - 1 + mu)^2 + y^2);
    
    % 动力学方程
    ax = 2*vy + x - (1 - mu) * (x + mu) / r1^3 - mu * (x - 1 + mu) / r2^3;
    ay = -2*vx + y - (1 - mu) * y / r1^3 - mu * y / r2^3;
    
    dstate = [vx; vy; ax; ay];
end

% -------------------------------------------------------------------------
% 事件监测函数：检测碰撞
% -------------------------------------------------------------------------
function [value, isterminal, direction] = detectCrash(~, state)
    global mu
    x = state(1);
    y = state(2);
    
    % 计算到两个主天体的距离
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - 1 + mu)^2 + y^2);
    
    % 设置一个极小的“物理半径”作为碰撞阈值
    % 如果距离小于这个值，认为撞上了，停止积分
    collision_radius = 1e-3; 
    
    % value(1) 监测与 P1 的距离，value(2) 监测与 P2 的距离
    % 当 value 穿过 0 时触发事件
    value = [r1 - collision_radius; r2 - collision_radius];
    
    isterminal = [1; 1]; % 1 表示触发时停止积分
    direction = [0; 0];  % 0 表示双向穿过零点都触发
end