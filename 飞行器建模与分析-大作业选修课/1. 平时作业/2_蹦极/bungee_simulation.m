function bungee_simulation
    % 参数
    m = 58;      % kg
    k = 30;      % N/m
    g = 10;      % m/s^2
    a1 = 1;
    a2 = 1;

    % 初始条件
    x0 = -30;   % m
    v0 = 0;     % m/s
    y0 = [x0; v0];

    % 仿真时间
    tspan = [0 40];

    % ODE 求解
    [t, y] = ode45(@(t, y) dynamics(t, y, m, k, g, a1, a2), tspan, y0);

    % 提取结果
    x = y(:,1);         % 位置
    v = y(:,2);         % 速度
    a = zeros(size(t)); % 加速度

    for i = 1:length(t)
        dydt = dynamics(t(i), y(i,:), m, k, g, a1, a2);
        a(i) = dydt(2); % 加速度
    end

    % 绘图
    figure;
    subplot(3,1,1);
    plot(t, x, 'b', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('位置 x (m)');
    title('位置-时间曲线');
    grid on;

    subplot(3,1,2);
    plot(t, v, 'r', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('速度 v (m/s)');
    title('速度-时间曲线');
    grid on;

    subplot(3,1,3);
    plot(t, a, 'g', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('加速度 a (m/s^2)');
    title('加速度-时间曲线');
    grid on;
end

% 动力学方程
function dydt = dynamics(~, y, m, k, g, a1, a2)
    x = y(1);
    v = y(2);

    % 弹力
    if x > 0
        bx = -k * x;
    else
        bx = 0;
    end

    % 加速度
    a = (m*g + bx - a1*v - a2*abs(v)*v) / m;

    dydt = [v; a];
end
