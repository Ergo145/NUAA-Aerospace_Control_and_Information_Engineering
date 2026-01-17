    %% 实验内容3：CRTBP 轨道仿真综合程序
    % 本程序包含两个部分：
    % Part 1: 固定质量比 mu=0.1，改变初始位置 x0 (复现图 12.5)
    % Part 2: 固定初始位置 x0=0.1，改变质量比 mu (复现图 12.6)
    
    clc; clear; close all;
    
    % 定义全局变量 mu，以便 ODE 函数调用
    global mu_global
    
    %% ====================================================================
    %  Part 1: 固定 mu=0.1, 改变 x0 (对应图 12.5) 
    % ====================================================================
    fprintf('正在运行 Part 1: 固定 mu=0.1, 改变 x0...\n');
    
    mu_global = 0.1;       % 固定质量比
    C_target = 3.4;        % 雅可比能量
    x0_list = [-0.2, 0, 0.2, 0.7]; % 待仿真的 x0 列表 
    t_span = [0, 15];      % 积分时间
    
    figure('Color', 'w', 'Name', '图12.5: 改变初始位置 x0', 'Position', [50, 100, 800, 600]);
    
    for i = 1:length(x0_list)
        x0 = x0_list(i);
        
        % 1. 计算初始速度 vy0
        y0 = 0; vx0 = 0;
        [vy0, success] = calculate_initial_velocity(x0, y0, C_target, mu_global);
        
        if ~success
            fprintf('  Part 1 Case %d: x0=%.2f 能量不足，跳过。\n', i, x0);
            continue;
        end
        
        % 2. 积分求解
        state0 = [x0; y0; vx0; vy0];
        options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @detectCrash);
        
        try
            [t, state, te, ye, ~] = ode45(@crtbp_ode, t_span, state0, options);
        catch ME
            warning('积分计算出错: %s', ME.message);
            continue;
        end
        
        % 3. 绘图
        subplot(2, 2, i);
        hold on;
        
        % 绘制背景（禁止区域）和轨迹
        plot_background(mu_global, C_target);
        plot(state(:,1), state(:,2), 'b', 'LineWidth', 1.2);
        
        % 标记起点和天体
        plot(x0, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
        plot(-mu_global, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 6); % P1
        plot(1-mu_global, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4); % P2
        
        % 标记碰撞点
        if ~isempty(te)
            plot(ye(:,1), ye(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        
        title(['(Part 1) x_0 = ', num2str(x0)]);
        xlabel('x'); ylabel('y');
        axis equal; xlim([-1.6, 1.6]); ylim([-1.6, 1.6]); grid on;
    end
    sgtitle(['图12.5: 固定 C=3.4, \mu=0.1, 改变 x_0']);
    
    %% ====================================================================
    %  Part 2: 固定 x0=0.1, 改变 mu (对应图 12.6) 
    % ====================================================================
    fprintf('正在运行 Part 2: 固定 x0=0.1, 改变 mu...\n');
    
    x0_fixed = 0.1;        % 固定初始位置
    mu_list = [0.03, 0.06, 0.1, 0.5]; % 待仿真的 mu 列表 
    t_span = [0, 18];      % 增加一点积分时间
    
    figure('Color', 'w', 'Name', '图12.6: 改变质量比 mu', 'Position', [900, 100, 800, 600]);
    
    for i = 1:length(mu_list)
        mu_global = mu_list(i); % 更新全局变量 mu
        
        % 1. 计算初始速度 (依赖当前的 mu)
        y0 = 0; vx0 = 0;
        [vy0, success] = calculate_initial_velocity(x0_fixed, y0, C_target, mu_global);
        
        if ~success
            fprintf('  Part 2 Case %d: mu=%.2f 能量不足，跳过。\n', i, mu_global);
            continue;
        end
        
        % 2. 积分求解
        state0 = [x0_fixed; y0; vx0; vy0];
        options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9, 'Events', @detectCrash);
        
        try
            [t, state, te, ye, ~] = ode45(@crtbp_ode, t_span, state0, options);
        catch ME
            warning('积分计算出错: %s', ME.message);
            continue;
        end
        
        % 3. 绘图
        subplot(2, 2, i);
        hold on;
        
        % 绘制背景（背景随 mu 变化）
        plot_background(mu_global, C_target);
        
        % 绘制轨迹plot
        plot(state(:,1), state(:,2), 'b', 'LineWidth', 1.2);
        
        % 标记起点和天体（位置随 mu 变化）
        plot(x0_fixed, y0, 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 5);
        plot(-mu_global, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 6); % P1
        plot(1-mu_global, 0, 'ko', 'MarkerFaceColor', 'r', 'MarkerSize', 4); % P2
        
        % 标记碰撞点
        if ~isempty(te)
            plot(ye(:,1), ye(:,2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        
        title(['(Part 2) \mu = ', num2str(mu_global)]);
        xlabel('x'); ylabel('y');
        axis equal; xlim([-1.6, 1.6]); ylim([-1.6, 1.6]); grid on;
    end
    sgtitle(['图12.6: 固定 C=3.4, x_0=0.1, 改变 \mu']);


%% ================= Helper Functions =================

function [vy0, success] = calculate_initial_velocity(x, y, C, mu)
    % 根据雅可比积分计算初始速度
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - 1 + mu)^2 + y^2);
    Omega = 0.5 * (x^2 + y^2) + (1 - mu) / r1 + mu / r2;
    
    v_sq = 2 * Omega - C;
    
    if v_sq < 0
        vy0 = 0;
        success = false;
    else
        vy0 = sqrt(v_sq);
        success = true;
    end
end

function plot_background(mu, C)
    % 绘制零速度曲线 (禁止区域边界)
    [X, Y] = meshgrid(-1.6:0.02:1.6, -1.6:0.02:1.6);
    R1 = sqrt((X + mu).^2 + Y.^2);
    R2 = sqrt((X - 1 + mu).^2 + Y.^2);
    W = 0.5 * (X.^2 + Y.^2) + (1 - mu) ./ R1 + mu ./ R2;
    
    contour(X, Y, 2*W, [C, C], 'k', 'LineWidth', 1);
end

function dstate = crtbp_ode(~, state)
    % CRTBP 动力学方程
    global mu_global
    mu = mu_global;
    
    x = state(1); y = state(2); vx = state(3); vy = state(4);
    
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - 1 + mu)^2 + y^2);
    
    ax = 2*vy + x - (1 - mu) * (x + mu) / r1^3 - mu * (x - 1 + mu) / r2^3;
    ay = -2*vx + y - (1 - mu) * y / r1^3 - mu * y / r2^3;
    
    dstate = [vx; vy; ax; ay];
end

function [value, isterminal, direction] = detectCrash(~, state)
    % 碰撞检测事件
    global mu_global
    mu = mu_global;
    
    x = state(1); y = state(2);
    r1 = sqrt((x + mu)^2 + y^2);
    r2 = sqrt((x - 1 + mu)^2 + y^2);
    
    % 物理碰撞半径设为 1e-3
    value = [r1 - 1e-3; r2 - 1e-3]; 
    isterminal = [1; 1]; % 停止积分
    direction = [0; 0];
end