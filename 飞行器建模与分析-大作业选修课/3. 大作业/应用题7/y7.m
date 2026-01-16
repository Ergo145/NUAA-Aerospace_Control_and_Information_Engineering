% 导弹追踪问题求解
clear; clc; close all;

% 参数设置
v_missile = 6;      % 导弹速度 6 m/s
v_target = 2;       % 乙舰速度 W m/s (假设值)

% 初始位置
x0_missile = 0; y0_missile = 0;     % 导弹初始位置 (0,0)
x0_target = 2; y0_target = 0;       % 乙舰初始位置 (2,0)

% 时间设置
dt = 0.01;          % 时间步长
t_max = 10;         % 最大仿真时间
t = 0:dt:t_max;     % 时间向量

% 初始化数组
x_missile = zeros(size(t));
y_missile = zeros(size(t));
x_target = zeros(size(t));
y_target = zeros(size(t));

% 初始条件
x_missile(1) = x0_missile;
y_missile(1) = y0_missile;
x_target(1) = x0_target;
y_target(1) = y0_target;

% 数值求解 - 欧拉法
for i = 2:length(t)
    % 乙舰位置 (沿y轴匀速运动)
    x_target(i) = x0_target;
    y_target(i) = y0_target + v_target * t(i);
    
    % 计算导弹与乙舰的相对位置
    dx = x_target(i-1) - x_missile(i-1);
    dy = y_target(i-1) - y_missile(i-1);
    
    % 计算导弹速度方向
    distance = sqrt(dx^2 + dy^2);
    
    % 导弹速度分量
    vx_missile = v_missile * dx / distance;
    vy_missile = v_missile * dy / distance;
    
    % 更新导弹位置
    x_missile(i) = x_missile(i-1) + vx_missile * dt;
    y_missile(i) = y_missile(i-1) + vy_missile * dt;
    
    % 检查是否击中 (距离小于阈值)
    current_distance = sqrt((x_target(i) - x_missile(i))^2 + ...
                           (y_target(i) - y_missile(i))^2);
    
    if current_distance < 0.01
        hit_time = t(i);
        fprintf('导弹击中乙舰！\n');
        fprintf('击中时间: %.2f 秒\n', hit_time);
        fprintf('乙舰行驶距离: %.2f 米\n', v_target * hit_time);
        fprintf('击中时乙舰位置: (%.2f, %.2f)\n', x_target(i), y_target(i));
        fprintf('击中时导弹位置: (%.2f, %.2f)\n', x_missile(i), y_missile(i));
        break;
    end
end

% 如果未击中，使用完整数据
if ~exist('hit_time', 'var')
    hit_time = t(end);
    fprintf('在 %.1f 秒内未击中目标\n', t_max);
end

% 绘制轨迹
figure;
plot(x_missile, y_missile, 'r-', 'LineWidth', 2); hold on;
plot(x_target, y_target, 'b--', 'LineWidth', 2);
plot(x0_missile, y0_missile, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(x0_target, y0_target, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');

if exist('hit_time', 'var') && hit_time < t_max
    hit_index = find(t <= hit_time, 1, 'last');
    plot(x_missile(hit_index), y_missile(hit_index), 'go', ...
         'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(x_target(hit_index), y_target(hit_index), 'go', ...
         'MarkerSize', 10, 'MarkerFaceColor', 'g');
end

xlabel('x 坐标 (m)');
ylabel('y 坐标 (m)');
title('导弹追踪乙舰轨迹');
legend('导弹轨迹', '乙舰轨迹', '导弹起始点', '乙舰起始点', '击中点', ...
       'Location', 'northwest');
grid on;
axis equal;

% 绘制距离随时间变化
figure;
distance = sqrt((x_target(1:length(x_missile)) - x_missile).^2 + ...
                (y_target(1:length(y_missile)) - y_missile).^2);
plot(t(1:length(distance)), distance, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('导弹与乙舰距离 (m)');
title('导弹与乙舰距离随时间变化');
grid on;

if exist('hit_time', 'var') && hit_time < t_max
    hold on;
    plot(hit_time, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(hit_time, 0.1, sprintf('击中时间: %.2f s', hit_time), ...
         'HorizontalAlignment', 'center');
end