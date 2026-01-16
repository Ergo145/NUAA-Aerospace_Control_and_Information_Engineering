clc; clear;

%% 初始化
m = 100;
g = 9.8;
rho = 1.225;
S = 0.01;
C_Q = 1;
C_Y = 0.2;
alpha = deg2rad(3);
P = 500;
Z = [0,0,50,deg2rad(30)];
dt = 0.01;
t=0;
zero_count=0;
flag = true;
Zout = [];

%% 仿真
while flag
    if t>=10
        P=0;
    end
    dZ1 = calculate_flight(Z,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ2 = calculate_flight(Z + 0.5*dt*dZ1,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ3 = calculate_flight(Z + 0.5*dt*dZ2,P,m,g,alpha,rho,S,C_Q,C_Y);
    dZ4 = calculate_flight(Z + dt*dZ3,P,m,g,alpha,rho,S,C_Q,C_Y);

    Z = Z + (dZ1 + 2*dZ2 + 2*dZ3 + dZ4)*dt/6;
    Zout = [Zout;Z];
    t = t+dt;
    if Z(2) < 1e-1
        zero_count=zero_count+1;
    end
    if zero_count > 1
        flag = false;
    end
end

%% 绘图 - 科研风格
Zout = [0,0,50,deg2rad(30); Zout];
tf = 0:dt:t;
tf = [tf, t+dt];

% 设置全局绘图参数
set(0,'DefaultAxesFontSize',12);
set(0,'DefaultTextFontSize',12);
set(0,'DefaultLineLineWidth',1.5);

% 创建图形窗口
figure('Position', [100, 100, 1200, 900], 'Color', 'white');

% 子图1: x位置随时间变化
subplot(3,2,1);
plot(tf, Zout(:,1), 'b-', 'LineWidth', 2);
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('水平位置 x (m)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
% 添加零线参考
hold on;
plot(xlim, [0,0], 'k--', 'LineWidth', 0.5, 'Color', [0.5 0.5 0.5]);
% 添加最终位置标注
text(0.02, 0.98, sprintf('x_{final} = %.1f m', Zout(end,1)), ...
     'Units', 'normalized', 'FontSize', 10, 'VerticalAlignment', 'top', ...
     'BackgroundColor', 'white', 'EdgeColor', 'blue', 'Margin', 2);

% 子图2: y位置随时间变化
subplot(3,2,2);
plot(tf, Zout(:,2), 'r-', 'LineWidth', 2);
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('垂直位置 y (m)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
% 标记关键点（如最大值）
[ymax, idx] = max(Zout(:,2));
hold on;
plot(tf(idx), ymax, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'red');
text(tf(idx), ymax, sprintf('  y_{max}=%.1fm', ymax), ...
     'FontSize', 9, 'VerticalAlignment', 'bottom');
% 添加落地时间标注
landing_time = tf(end);
if ~isempty(landing_time)
    text(0.02, 0.85, sprintf('落地时间: %.1f s', landing_time), ...
         'Units', 'normalized', 'FontSize', 10, 'VerticalAlignment', 'top', ...
         'BackgroundColor', 'white', 'EdgeColor', 'red', 'Margin', 2);
end

% 子图3: 速度随时间变化
subplot(3,2,3);
plot(tf, Zout(:,3), 'color', [0, 0.5, 0], 'LineWidth', 2); % 深绿色
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('速度 v (m/s)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
% 添加初始和最终速度标注
hold on;
plot(tf(1), Zout(1,3), 'go', 'MarkerSize', 6, 'MarkerFaceColor', 'green');
plot(tf(end), Zout(end,3), 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'red');
text(tf(1), Zout(1,3), sprintf('  v_0=%.1f', Zout(1,3)), ...
     'FontSize', 9, 'VerticalAlignment', 'bottom');
text(tf(end), Zout(end,3), sprintf('  v_f=%.1f', Zout(end,3)), ...
     'FontSize', 9, 'VerticalAlignment', 'top');
% 标记速度极值
[vmax, vmax_idx] = max(Zout(:,3));
[vmin, vmin_idx] = min(Zout(:,3));
text(0.02, 0.15, sprintf('v_{max} = %.1f m/s\nv_{min} = %.1f m/s', vmax, vmin), ...
     'Units', 'normalized', 'FontSize', 9, 'VerticalAlignment', 'bottom', ...
     'BackgroundColor', 'white');

% 子图4: 俯仰角随时间变化
subplot(3,2,4);
plot(tf, rad2deg(Zout(:,4)), 'color', [0.7, 0, 0.7], 'LineWidth', 2); % 紫色
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('俯仰角 \theta (°)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
% 添加角度变化范围标注
theta_range = sprintf('\\theta: %.1f° → %.1f°', rad2deg(Zout(1,4)), rad2deg(Zout(end,4)));
text(0.02, 0.98, theta_range, 'Units', 'normalized', ...
     'FontSize', 10, 'VerticalAlignment', 'top', ...
     'BackgroundColor', 'white', 'EdgeColor', [0.7, 0, 0.7], 'Margin', 2);
% 标记角度极值
[thetamax, theta_max_idx] = max(rad2deg(Zout(:,4)));
[thetamin, theta_min_idx] = min(rad2deg(Zout(:,4)));
hold on;
plot(tf(theta_max_idx), thetamax, '^', 'MarkerSize', 5, ...
     'MarkerFaceColor', [0.7, 0, 0.7], 'Color', [0.7, 0, 0.7]);
plot(tf(theta_min_idx), thetamin, 'v', 'MarkerSize', 5, ...
     'MarkerFaceColor', [0.7, 0, 0.7], 'Color', [0.7, 0, 0.7]);

% 子图5: 推力随时间变化
subplot(3,2,5);
% 创建推力数据
p = zeros(size(tf));
p(tf <= 10 & tf > 0) = 500;
p(1) = 0;

plot(tf, p, 'k-', 'LineWidth', 2);
xlabel('时间 t (s)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('推力 P (N)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
% 标记推力关闭时刻
hold on;
plot([10,10], ylim, 'r--', 'LineWidth', 1.2);
text(10, 250, '推力关闭', 'Rotation', 90, 'FontSize', 10, ...
     'HorizontalAlignment', 'center', 'BackgroundColor', 'white', ...
     'EdgeColor', 'red', 'Margin', 2);
% 添加推力统计
text(0.02, 0.85, sprintf('推力持续时间: 0-10 s\n最大推力: 500 N'), ...
     'Units', 'normalized', 'FontSize', 9, 'VerticalAlignment', 'top', ...
     'BackgroundColor', 'white');

% 子图6: 轨迹图
subplot(3,2,6);
plot(Zout(:,1), Zout(:,2), 'b-', 'LineWidth', 2);
hold on;
% 标记起点和终点
plot(Zout(1,1), Zout(1,2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'green', ...
     'MarkerEdgeColor', 'black');
plot(Zout(end,1), Zout(end,2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red', ...
     'MarkerEdgeColor', 'black');

% 添加轨迹方向指示器（每隔一定距离）
num_arrows = 5;
arrow_indices = round(linspace(1, length(Zout), num_arrows + 2));
arrow_indices = arrow_indices(2:end-1); % 去掉首尾点

for i = arrow_indices
    if i <= length(Zout) && i > 1
        quiver(Zout(i,1), Zout(i,2), ...
               cos(Zout(i,4)), sin(Zout(i,4)), ...
               8, 'k', 'LineWidth', 1.2, 'MaxHeadSize', 1.5, ...
               'AutoScale', 'off');
    end
end

xlabel('水平位置 x (m)', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('垂直位置 y (m)', 'FontSize', 11, 'FontWeight', 'bold');
grid on;
set(gca, 'GridAlpha', 0.3, 'GridLineStyle', '--');
box on;
axis equal;

% 添加图例
legend('轨迹', '起点', '终点', '方向', 'Location', 'best', ...
       'FontSize', 9, 'Box', 'on');

% 添加轨迹信息
info_text = sprintf('轨迹信息:\n水平距离: %.1f m\n最大高度: %.1f m\n飞行时间: %.1f s', ...
                   Zout(end,1), max(Zout(:,2)), tf(end));
text(0.02, 0.98, info_text, 'Units', 'normalized', ...
     'FontSize', 9, 'VerticalAlignment', 'top', ...
     'BackgroundColor', 'white', 'EdgeColor', 'black', 'Margin', 3);

% 添加总标题和注释
sgtitle('飞行器轨迹与状态参数仿真结果', 'FontSize', 14, 'FontWeight', 'bold');

% 添加仿真参数注释框
annotation('textbox', [0.02, 0.02, 0.25, 0.12], 'String', ...
    sprintf('仿真参数:\\n质量: m = %.0f kg\\n推力: P = %.0f N\\n攻角: α = %.1f°\\n时间步长: dt = %.3f s\\n初始速度: v₀ = %.0f m/s', ...
    m, 500, rad2deg(alpha), dt, Zout(1,3)), ...
    'FontSize', 9, 'BackgroundColor', [0.95, 0.95, 0.95], ...
    'EdgeColor', 'black', 'FitBoxToText', 'on', 'VerticalAlignment', 'bottom');

% 添加气动参数注释框
annotation('textbox', [0.28, 0.02, 0.25, 0.12], 'String', ...
    sprintf('气动参数:\\n阻力系数: C_Q = %.1f\\n升力系数: C_Y = %.1f\\n参考面积: S = %.3f m²\\n空气密度: ρ = %.3f kg/m³', ...
    C_Q, C_Y, S, rho), ...
    'FontSize', 9, 'BackgroundColor', [0.95, 0.95, 0.95], ...
    'EdgeColor', 'black', 'FitBoxToText', 'on', 'VerticalAlignment', 'bottom');

% 保存图片（可选）
% print('flight_simulation_results', '-dpng', '-r300');

%% 函数
% 飞行力学
% Z = [x y v theta]
function dZ = calculate_flight(Z,P,m,g,alpha,rho,S,C_Q,C_Y)
    x = Z(1);
    y = Z(2);
    v = Z(3);
    theta = Z(4);
    [Q,Y] = calculate_qidong(v,rho,S,C_Q,C_Y);
    dx = v*cos(theta);
    dy = v*sin(theta);
    dv = -g*sin(theta) - Q/m + P*cos(alpha);
    dtheta = (-g*cos(theta) + Y/m + P*sin(alpha)/m)/v;
    
    dZ = [dx,dy,dv,dtheta];
end

% 气动参数计算
function [Q,Y] = calculate_qidong(v,rho,S,C_Q,C_Y)
    Q = 0.5*rho*(v^2)*S*C_Q;
    Y = 0.5*rho*(v^2)*S*C_Y;
end