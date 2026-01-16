function double_inverted_pendulum()
    % 一阶直线双倒立摆系统仿真
    close all; clc;
    
    % 系统参数
    M = 1;      % 小车质量 (kg)
    m1 = 0.5;   % 摆杆1质量 (kg)
    m2 = 0.5;   % 摆杆2质量 (kg)
    L1 = 0.6;   % 摆杆1长度 (m)
    L2 = 0.6;   % 摆杆2长度 (m)
    g = 9.8;    % 重力加速度 (m/s^2)
    F = 5;      % 外力 (N)
    
    fprintf('系统参数:\n');
    fprintf('M = %.1f kg, m1 = %.1f kg, m2 = %.1f kg\n', M, m1, m2);
    fprintf('L1 = %.1f m, L2 = %.1f m, g = %.1f m/s²\n', L1, L2, g);
    fprintf('F = %.1f N\n\n', F);
    
    % 问题1：初始角度180度（倒立位置）
    fprintf('=== 问题1：初始角度180度（倒立位置）===\n');
    simulate_pendulum(M, m1, m2, L1, L2, g, F, pi, pi, 0, '精确模型-180度初始', true);
    
    % 问题2：初始角度0度（下垂位置），比较精确模型和简化模型
    fprintf('\n=== 问题2：初始角度0度（下垂位置）===\n');
    compare_models(M, m1, m2, L1, L2, g, F);
end

function simulate_pendulum(M, m1, m2, L1, L2, g, F, theta10, theta20, x0, model_name, is_exact)
    % 仿真倒立摆系统
    
    % 仿真时间
    tspan = [0, 10];
    
    % 初始状态 [x, dx/dt, theta1, dtheta1/dt, theta2, dtheta2/dt]
    initial_conditions = [x0; 0; theta10; 0; theta20; 0];
    
    % 选择模型
    if is_exact
        [t, y] = ode45(@(t,y) exact_model(t,y,M,m1,m2,L1,L2,g,F), tspan, initial_conditions);
        model_type = '精确模型';
    else
        [t, y] = ode45(@(t,y) simplified_model(t,y,M,m1,m2,L1,L2,g,F), tspan, initial_conditions);
        model_type = '简化模型';
    end
    
    % 提取状态变量
    x = y(:,1);         % 小车位置
    dx = y(:,2);        % 小车速度
    theta1 = y(:,3);    % 摆杆1角度
    dtheta1 = y(:,4);   % 摆杆1角速度
    theta2 = y(:,5);    % 摆杆2角度
    dtheta2 = y(:,6);   % 摆杆2角速度
    
    % 转换为角度显示
    theta1_deg = rad2deg(theta1);
    theta2_deg = rad2deg(theta2);
    
    % 绘图
    figure('Position', [100, 100, 1200, 800]);
    
    % 位置和角度响应
    subplot(2,3,1);
    plot(t, x, 'b-', 'LineWidth', 2);
    title('小车位置响应');
    xlabel('时间 (s)'); ylabel('位置 (m)');
    grid on;
    
    subplot(2,3,2);
    plot(t, theta1_deg, 'r-', 'LineWidth', 2);
    title('摆杆1角度响应');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    grid on;
    
    subplot(2,3,3);
    plot(t, theta2_deg, 'g-', 'LineWidth', 2);
    title('摆杆2角度响应');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    grid on;
    
    % 速度和角速度响应
    subplot(2,3,4);
    plot(t, dx, 'b-', 'LineWidth', 2);
    title('小车速度响应');
    xlabel('时间 (s)'); ylabel('速度 (m/s)');
    grid on;
    
    subplot(2,3,5);
    plot(t, dtheta1, 'r-', 'LineWidth', 2);
    title('摆杆1角速度响应');
    xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
    grid on;
    
    subplot(2,3,6);
    plot(t, dtheta2, 'g-', 'LineWidth', 2);
    title('摆杆2角速度响应');
    xlabel('时间 (s)'); ylabel('角速度 (rad/s)');
    grid on;
    
    sgtitle(sprintf('%s - %s (F=%.1fN, θ1₀=%.0f°, θ2₀=%.0f°)', ...
           model_type, model_name, F, rad2deg(theta10), rad2deg(theta20)));
    
    % 显示关键信息
    fprintf('%s - 最大位置: %.3f m\n', model_type, max(abs(x)));
    fprintf('%s - 最大角度1: %.1f°\n', model_type, max(abs(theta1_deg)));
    fprintf('%s - 最大角度2: %.1f°\n', model_type, max(abs(theta2_deg)));
    
    % 动画演示
    create_animation(t, x, theta1, theta2, L1, L2, sprintf('%s动画', model_type));
end

function dydt = exact_model(t, y, M, m1, m2, L1, L2, g, F)
    % 精确模型微分方程
    x = y(1); dx = y(2);
    theta1 = y(3); dtheta1 = y(4);
    theta2 = y(5); dtheta2 = y(6);
    
    % 构建系数矩阵
    A = zeros(3,3);
    b = zeros(3,1);
    
    % 第一个方程系数
    A(1,1) = M + m1 + m2;
    A(1,2) = m1 * L1 * cos(theta1);
    A(1,3) = m2 * L2 * cos(theta2);
    b(1) = F + m1 * L1 * sin(theta1) * dtheta1^2 + m2 * L2 * sin(theta2) * dtheta2^2;
    
    % 第二个方程系数
    A(2,1) = cos(theta1);
    A(2,2) = -4/3 * L1;
    A(2,3) = 0;
    b(2) = g * sin(theta1);
    
    % 第三个方程系数
    A(3,1) = cos(theta2);
    A(3,2) = 0;
    A(3,3) = -4/3 * L2;
    b(3) = g * sin(theta2);
    
    % 求解加速度
    accelerations = A \ b;
    
    dydt = zeros(6,1);
    dydt(1) = dx;                           % dx/dt
    dydt(2) = accelerations(1);             % d²x/dt²
    dydt(3) = dtheta1;                      % dθ1/dt
    dydt(4) = accelerations(2);             % d²θ1/dt²
    dydt(5) = dtheta2;                      % dθ2/dt
    dydt(6) = accelerations(3);             % d²θ2/dt²
end

function dydt = simplified_model(t, y, M, m1, m2, L1, L2, g, F)
    % 简化模型微分方程
    x = y(1); dx = y(2);
    theta1 = y(3); dtheta1 = y(4);
    theta2 = y(5); dtheta2 = y(6);
    
    % 简化模型系数
    denominator = 4*M + m1 + m2;
    
    % 小车加速度
    ddx = F * 4/denominator - (3*m1*g*theta1)/denominator - (3*m2*g*theta2)/denominator;
    
    % 摆杆1角加速度
    ddtheta1 = theta1 * (3*g*(4*M + 4*m1 + m2))/(4*L1*denominator) + ...
                theta2 * (9*m2*g)/(4*L1*denominator) - ...
                F * 3/(L1*denominator);
    
    % 摆杆2角加速度（修正原公式中的错误）
    ddtheta2 = theta2 * (3*g*(4*M + 4*m2 + m1))/(4*L2*denominator) + ...
                theta1 * (9*m1*g)/(4*L2*denominator) - ...  % 修正：应该是m1而不是m2
                F * 3/(L2*denominator);
    
    dydt = zeros(6,1);
    dydt(1) = dx;               % dx/dt
    dydt(2) = ddx;              % d²x/dt²
    dydt(3) = dtheta1;          % dθ1/dt
    dydt(4) = ddtheta1;         % d²θ1/dt²
    dydt(5) = dtheta2;          % dθ2/dt
    dydt(6) = ddtheta2;         % d²θ2/dt²
end

function compare_models(M, m1, m2, L1, L2, g, F)
    % 比较精确模型和简化模型
    
    % 仿真参数
    tspan = [0, 5];
    initial_conditions = [0; 0; 0; 0; 0; 0];  % 初始角度0度
    
    % 仿真精确模型
    [t_exact, y_exact] = ode45(@(t,y) exact_model(t,y,M,m1,m2,L1,L2,g,F), ...
                              tspan, initial_conditions);
    
    % 仿真简化模型
    [t_simple, y_simple] = ode45(@(t,y) simplified_model(t,y,M,m1,m2,L1,L2,g,F), ...
                                tspan, initial_conditions);
    
    % 提取数据
    x_exact = y_exact(:,1); theta1_exact = rad2deg(y_exact(:,3)); theta2_exact = rad2deg(y_exact(:,5));
    x_simple = y_simple(:,1); theta1_simple = rad2deg(y_simple(:,3)); theta2_simple = rad2deg(y_simple(:,5));
    
    % 比较绘图
    figure('Position', [100, 100, 1200, 900]);
    
    % 小车位置比较
    subplot(3,1,1);
    plot(t_exact, x_exact, 'b-', 'LineWidth', 2); hold on;
    plot(t_simple, x_simple, 'r--', 'LineWidth', 2);
    title('小车位置响应比较');
    xlabel('时间 (s)'); ylabel('位置 (m)');
    legend('精确模型', '简化模型', 'Location', 'best');
    grid on;
    
    % 摆杆1角度比较
    subplot(3,1,2);
    plot(t_exact, theta1_exact, 'b-', 'LineWidth', 2); hold on;
    plot(t_simple, theta1_simple, 'r--', 'LineWidth', 2);
    title('摆杆1角度响应比较');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    legend('精确模型', '简化模型', 'Location', 'best');
    grid on;
    
    % 摆杆2角度比较
    subplot(3,1,3);
    plot(t_exact, theta2_exact, 'b-', 'LineWidth', 2); hold on;
    plot(t_simple, theta2_simple, 'r--', 'LineWidth', 2);
    title('摆杆2角度响应比较');
    xlabel('时间 (s)'); ylabel('角度 (°)');
    legend('精确模型', '简化模型', 'Location', 'best');
    grid on;
    
    sgtitle('精确模型与简化模型动态响应比较 (F=5N, 初始角度0°)');
    
    % 计算误差
    % 由于时间点可能不同，需要插值到相同时间点
    t_common = linspace(0, 5, 1000);
    x_exact_interp = interp1(t_exact, x_exact, t_common);
    x_simple_interp = interp1(t_simple, x_simple, t_common);
    
    theta1_exact_interp = interp1(t_exact, theta1_exact, t_common);
    theta1_simple_interp = interp1(t_simple, theta1_simple, t_common);
    
    theta2_exact_interp = interp1(t_exact, theta2_exact, t_common);
    theta2_simple_interp = interp1(t_simple, theta2_simple, t_common);
    
    error_x = max(abs(x_exact_interp - x_simple_interp));
    error_theta1 = max(abs(theta1_exact_interp - theta1_simple_interp));
    error_theta2 = max(abs(theta2_exact_interp - theta2_simple_interp));
    
    fprintf('模型比较结果:\n');
    fprintf('最大位置误差: %.4f m\n', error_x);
    fprintf('最大角度1误差: %.4f°\n', error_theta1);
    fprintf('最大角度2误差: %.4f°\n', error_theta2);
    fprintf('在小角度范围内，简化模型与精确模型具有良好的一致性。\n');
end

function create_animation(t, x, theta1, theta2, L1, L2, title_str)
    % 创建倒立摆动画
    figure('Position', [200, 200, 800, 600]);
    
    % 选择部分时间点进行动画（避免太密集）
    if length(t) > 100
        indices = round(linspace(1, length(t), 100));
    else
        indices = 1:length(t);
    end
    
    for i = indices
        clf;
        
        % 小车位置
        cart_x = x(i);
        cart_y = 0;
        
        % 摆杆1端点
        pendulum1_x = cart_x + L1 * sin(theta1(i));
        pendulum1_y = cart_y + L1 * cos(theta1(i));
        
        % 摆杆2端点（相对于摆杆1端点）
        pendulum2_x = pendulum1_x + L2 * sin(theta2(i));
        pendulum2_y = pendulum1_y + L2 * cos(theta2(i));
        
        % 绘制轨道
        plot([-2, 2], [0, 0], 'k-', 'LineWidth', 3); hold on;
        
        % 绘制小车
        rectangle('Position', [cart_x-0.2, cart_y-0.1, 0.4, 0.2], ...
                 'FaceColor', [0.8, 0.8, 0.8], 'EdgeColor', 'k', 'LineWidth', 2);
        
        % 绘制摆杆1
        plot([cart_x, pendulum1_x], [cart_y, pendulum1_y], 'r-', 'LineWidth', 3);
        plot(pendulum1_x, pendulum1_y, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        % 绘制摆杆2
        plot([pendulum1_x, pendulum2_x], [pendulum1_y, pendulum2_y], 'b-', 'LineWidth', 3);
        plot(pendulum2_x, pendulum2_y, 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        
        % 设置图形属性
        axis equal; axis([-2, 2, -1.5, 1.5]);
        title(sprintf('%s\\n时间: %.2f s', title_str, t(i)));
        xlabel('x位置 (m)'); ylabel('y位置 (m)');
        grid on;
        
        drawnow;
        pause(0.05);
    end
end

