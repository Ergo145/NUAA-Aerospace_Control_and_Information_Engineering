function bouncy_ball_bounces
% BOUNCY_BALL_BOUNCES  模拟有恢复系数的弹力球重复反弹
% 初始条件：从高度 h0 以速度 v0（向上为正）抛出
% 每次撞地时速度按 v_new = e * |v_impact| 向上重设（等价 v_new = -e*v_impact）
%
% 输出：绘制位置、速度、加速度随时间曲线

% 参数
g = 9.81;      % 重力，加速度 (m/s^2)
e = 0.8;       % 恢复系数（弹性系数）
h0 = 10.0;     % 初始高度 (m)
v0 = 15.0;     % 初始速度 (m/s) 向上为正
t_max = 20;    % 最大仿真时间 (s)
max_bounces = 50;   % 最大允许反弹次数（防止死循环）
pts_per_flight = 200; % 每次飞行段的采样点数

% 初始化记录向量
T = []; H = []; V = []; A = [];

t_global = 0;
h_curr = h0;
v_curr = v0;
bounce_count = 0;

while t_global < t_max && bounce_count < max_bounces
    % 求当前飞行段撞地时间（正根）
    % 高度解析式: h(t) = h_curr + v_curr*t - 0.5*g*t^2
    % 求 h(t_hit) = 0 -> 0.5*g*t^2 - v_curr*t - h_curr = 0
    a_q = 0.5 * g;
    b_q = -v_curr;
    c_q = -h_curr;
    D = b_q^2 - 4*a_q*c_q;
    if D < 0
        % 不会再落地（理论上不会发生在本问题），将剩余时间用单段表示并退出
        t_remain = t_max - t_global;
        t_seg = linspace(0, t_remain, pts_per_flight);
        h_seg = h_curr + v_curr.*t_seg - 0.5*g.*t_seg.^2;
        v_seg = v_curr - g.*t_seg;
        a_seg = -g * ones(size(t_seg));
        T = [T; t_global + t_seg(:)];
        H = [H; h_seg(:)];
        V = [V; v_seg(:)];
        A = [A; a_seg(:)];
        break;
    end
    t_hit1 = (-b_q + sqrt(D)) / (2*a_q);
    t_hit2 = (-b_q - sqrt(D)) / (2*a_q);
    % 取正且大于很小值的根
    ts = sort([t_hit1, t_hit2]);
    t_hit = ts(find(ts > 1e-10,1,'first'));
    if isempty(t_hit)
        % 没有正根，退出
        break;
    end

    % 若撞地时间超出仿真截止，则只取到仿真截止
    if t_global + t_hit > t_max
        t_seg = linspace(0, t_max - t_global, pts_per_flight);
        h_seg = h_curr + v_curr.*t_seg - 0.5*g.*t_seg.^2;
        v_seg = v_curr - g.*t_seg;
        a_seg = -g * ones(size(t_seg));
        T = [T; t_global + t_seg(:)];
        H = [H; h_seg(:)];
        V = [V; v_seg(:)];
        A = [A; a_seg(:)];
        break;
    end

    % 记录该飞行段（从 t=0 到 t=t_hit）
    t_seg = linspace(0, t_hit, pts_per_flight);
    h_seg = h_curr + v_curr.*t_seg - 0.5*g.*t_seg.^2;
    v_seg = v_curr - g.*t_seg;
    a_seg = -g * ones(size(t_seg));

    T = [T; t_global + t_seg(:)];
    H = [H; h_seg(:)];
    V = [V; v_seg(:)];
    A = [A; a_seg(:)];

    % 更新到撞击时刻
    t_global = t_global + t_hit;
    v_impact = v_curr - g * t_hit; % 这应为负（向下）
    h_impact = 0;                   % 地面
    % 重设速度（向上为正）
    v_after = -e * v_impact; % = e*|v_impact|

    % 若碰撞后速度极小则停止
    if abs(v_after) < 1e-3
        % 在地面停止，记录一个点并退出
        T = [T; t_global];
        H = [H; 0];
        V = [V; 0];
        A = [A; 0];
        break;
    end

    % 准备下一段飞行
    bounce_count = bounce_count + 1;
    h_curr = 0;
    v_curr = v_after;

    % 小的停顿点（可选）-- 在时间序列中加入撞地瞬间的点，表明速度跳变
    % 用两个点来表现撞击前后速度的突变
    T = [T; t_global];
    H = [H; 0];
    V = [V; v_after];   % 把碰撞后新的速度记录在同一时间点（瞬态）
    A = [A; NaN];       % 撞击瞬间加速度可以视为冲击，标为 NaN（绘图时会断开）
end

% 若最后时间不足 t_max，可在末尾补齐常加速度段（自由落体）
if t_global < t_max
    t_rem = t_max - t_global;
    t_seg = linspace(0, t_rem, pts_per_flight);
    h_seg = h_curr + v_curr.*t_seg - 0.5*g.*t_seg.^2;
    v_seg = v_curr - g.*t_seg;
    a_seg = -g * ones(size(t_seg));
    T = [T; t_global + t_seg(:)];
    H = [H; h_seg(:)];
    V = [V; v_seg(:)];
    A = [A; a_seg(:)];
end

% 绘图
figure('Name','弹力球反弹仿真','NumberTitle','off','Position',[200 200 800 700]);

subplot(3,1,1);
plot(T, H,'LineWidth',1.2);
ylabel('高度 h (m)');
title(sprintf('弹力球反弹 仿真 (e=%.2f, h_0=%.1f m, v_0=%.1f m/s)', e, h0, v0));
grid on;

subplot(3,1,2);
plot(T, V,'LineWidth',1.2);
ylabel('速度 v (m/s)');
grid on;

subplot(3,1,3);
% 将 NaN 作为中断，绘制加速度（撞击瞬间设为 NaN）
plot(T, A,'LineWidth',1.2);
xlabel('时间 t (s)');
ylabel('加速度 a (m/s^2)');
grid on;

% optional: print bounce count
fprintf('仿真结束：总反弹次数 = %d, 仿真总时间 = %.3f s\n', bounce_count, T(end));
end
