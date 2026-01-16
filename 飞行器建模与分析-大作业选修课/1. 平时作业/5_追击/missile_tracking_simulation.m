function missile_tracking_simulation()
    % 导弹追踪仿真主函数
    clc;clear;close
    % 参数设置
    w_values = [20, 5];  % 导弹速度
    tspan = [0, 50];     % 仿真时间
    
    % 创建图形窗口
    figure('Position', [100, 100, 1200, 800]);
    
    % 计算完整椭圆轨迹
    t_full = linspace(0, 2*pi, 1000);
    x_v_full = 10 + 20*cos(t_full);
    y_v_full = 20 + 5*sin(t_full);
    
    % 仿真不同速度的导弹轨迹
    for i = 1:length(w_values)
        w = w_values(i);
        
        % 使用ode45求解微分方程
        options = odeset('Events', @events);
        [t, y, te, ye, ie] = ode45(@(t,y) missile_eq(t,y,w), tspan, [0;0], options);
        
        % 提取导弹轨迹
        x_m = y(:,1);
        y_m = y(:,2);
        
        % 计算飞行器轨迹（对应的时间点）
        x_v = 10 + 20*cos(t);
        y_v = 20 + 5*sin(t);
        
        % 绘图
        subplot(2,2,i);
        % 先画完整椭圆
        plot(x_v_full, y_v_full, 'k--', 'LineWidth', 1, 'Color', [0.7 0.7 0.7]);
        hold on;
        % 再画实际飞行轨迹和导弹轨迹
        plot(x_m, y_m, 'r-', 'LineWidth', 2); 
        plot(x_v, y_v, 'b-', 'LineWidth', 1.5);
        plot(x_m(1), y_m(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        
        if ~isempty(te)
            plot(x_m(end), y_m(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            plot(x_v(end), y_v(end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        else
            plot(x_m(end), y_m(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
            plot(x_v(end), y_v(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
        end
        
        plot(x_v(1), y_v(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
        
        title(sprintf('导弹速度 w = %.1f', w));
        xlabel('x坐标'); ylabel('y坐标');
        legend('完整椭圆', '导弹轨迹', '飞行器轨迹', '导弹起点', '导弹终点', '飞行器终点', '飞行器起点', ...
               'Location', 'best');
        grid on; axis equal;
        
        % 显示相遇信息
        if ~isempty(te)
            fprintf('w = %.1f: 在 t = %.2f 时刻相遇，位置 (%.2f, %.2f)\n', ...
                    w, te, ye(1), ye(2));
        else
            fprintf('w = %.1f: 在给定时间内未相遇\n', w);
        end
    end
    
    % 寻找最小追踪速度
    find_min_speed(x_v_full, y_v_full);
end

function dydt = missile_eq(t, y, w)
    % 导弹运动微分方程
    x_m = y(1); y_m = y(2);
    
    % 飞行器位置
    x_v = 10 + 20*cos(t);
    y_v = 20 + 5*sin(t);
    
    % 计算距离
    D = sqrt((x_v - x_m)^2 + (y_v - y_m)^2);
    
    % 避免除零
    if D < 1e-3
        dydt = [0; 0];
    else
        dydt = [w*(x_v - x_m)/D; w*(y_v - y_m)/D];
    end
end

function [value, isterminal, direction] = events(t, y)
    % 事件函数：检测导弹与飞行器相遇
    x_m = y(1); y_m = y(2);
    x_v = 10 + 20*cos(t);
    y_v = 20 + 5*sin(t);
    
    % 相遇条件：距离小于0.1
    value = sqrt((x_v - x_m)^2 + (y_v - y_m)^2) - 0.1;
    isterminal = 1;  % 相遇时停止仿真
    direction = -1;  % 距离减小时触发
end

function find_min_speed(x_v_full, y_v_full)
    % 寻找最小追踪速度
    fprintf('\n寻找最小追踪速度...\n');
    
    % 二分法搜索最小速度
    w_low = 1.0;
    w_high = 10.0;
    tolerance = 0.01;
    max_iter = 20;
    
    for iter = 1:max_iter
        w_mid = (w_low + w_high) / 2;
        
        % 测试当前速度是否能追上（使用事件函数）
        options = odeset('Events', @events);
        sol = ode45(@(t,y) missile_eq(t,y,w_mid), [0, 200], [0;0], options);
        
        % 检查是否相遇
        if ~isempty(sol.ie)
            % 能追上，尝试更小的速度
            w_high = w_mid;
            fprintf('迭代 %d: w = %.3f - 能追上\n', iter, w_mid);
        else
            % 不能追上，需要更大的速度
            w_low = w_mid;
            fprintf('迭代 %d: w = %.3f - 不能追上\n', iter, w_mid);
        end
        
        if (w_high - w_low) < tolerance
            break;
        end
    end
    
    min_speed = (w_low + w_high) / 2;
    fprintf('\n最小追踪速度约为: %.3f\n', min_speed);
    
    % 验证最小速度
    verify_min_speed(min_speed, x_v_full, y_v_full);
end

function verify_min_speed(w_min, x_v_full, y_v_full)
    % 验证最小追踪速度
    subplot(2,2,3:4);
    
    % 测试略大于最小速度的情况
    w_test = w_min * 1.01;
    options = odeset('Events', @events);
    sol = ode45(@(t,y) missile_eq(t,y,w_test), [0, 200], [0;0], options);
    
    % 提取轨迹
    t_eval = linspace(0, sol.x(end), 1000);
    y_eval = deval(sol, t_eval);
    x_m = y_eval(1,:)';
    y_m = y_eval(2,:)';
    
    % 计算对应的飞行器位置
    x_v = 10 + 20*cos(t_eval);
    y_v = 20 + 5*sin(t_eval);
    
    % 绘图
    % 先画完整椭圆
    plot(x_v_full, y_v_full, 'k--', 'LineWidth', 1, 'Color', [0.7 0.7 0.7]);
    hold on;
    % 再画导弹和飞行器轨迹
    plot(x_m, y_m, 'r-', 'LineWidth', 2); 
    plot(x_v, y_v, 'b-', 'LineWidth', 1.5);
    plot(x_m(1), y_m(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    if ~isempty(sol.ie)
        plot(x_m(end), y_m(end), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
        plot(x_v(end), y_v(end), 'bs', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
        title(sprintf('最小追踪速度验证: w = %.3f (成功追踪)', w_test));
    else
        plot(x_m(end), y_m(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
        plot(x_v(end), y_v(end), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
        title(sprintf('最小追踪速度验证: w = %.3f (未追上)', w_test));
    end
    
    plot(x_v(1), y_v(1), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
    
    xlabel('x坐标'); ylabel('y坐标');
    legend('完整椭圆', '导弹轨迹', '飞行器轨迹', '导弹起点', '导弹终点', '飞行器终点', '飞行器起点', ...
           'Location', 'best');
    grid on; axis equal;
    
    if ~isempty(sol.ie)
        fprintf('验证: w = %.3f 时成功追踪\n', w_test);
    else
        %fprintf('验证: w = %.3f 时未追上\n', w_test);
    end
end

