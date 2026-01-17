clc;clear;close all;

%% 1. 定义轨道六根数和物理常数
a = 10000;      % 半长轴 (km)
e = 0.3;        % 偏心率 (0 < e < 1)
i_deg = 45;     % 轨道倾角 (degrees)
Omega_deg = 60; % 升交点赤经 (degrees)
omega_deg = 30; % 近地点幅角 (degrees)
M0_deg = 45;    % 卫星当前的初始平近点角
% 物理常数
mu = 398600;    % 地球引力常数 (km^3/s^2)
Re = 6378;      % 地球半径 (km)
% 辅助计算轨道周期 T，用于数据展示
T = 2 * pi * sqrt(a^3 / mu); % 轨道周期 (秒)

%% 2. 进行 SIMULINK 仿真
open_system('dynamic.slx');     %打开系统
simout = sim("dynamic.slx");    %运行系统
t = simout.tout;
r_raw = simout.r.Data; 
v_raw = simout.v.Data; 
r_plot = squeeze(r_raw);
v_plot = squeeze(v_raw);
N = size(r_plot, 2);
%% 3. 准备绘图环境
figure('Color', 'w', 'Position', [100, 100, 1000, 600]); 
hold on; 
axis equal; 
grid on;
view(120, 25); % 设置视角
xlabel('X (km)'); 
ylabel('Y (km)'); 
zlabel('Z (km)');
title('航天器轨道三维仿真与 Simulink 数据动画');
% 设置坐标轴范围
max_extent = a * (1 + e) * 1.1; 
xlim([-max_extent, max_extent]);
ylim([-max_extent, max_extent]);
zlim([-max_extent, max_extent]);
% 绘制地球
[x_e, y_e, z_e] = sphere(50);
surf(x_e*Re, y_e*Re, z_e*Re, 'FaceColor', [0.9, 0.9, 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);

%% 4. 绘制坐标系、赤道平面和完整轨迹
% 绘制坐标轴
L = a * 1.8; % 坐标轴长度
arrow3d([0 0 0], [L 0 0], 'r', 2); % X轴
text(L*1.05, 0, 0, '春分点 (X)', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
arrow3d([0 0 0], [0 L 0], [0 0.5 0], 1); % Y轴
text(0, L*1.05, 0, 'Y', 'Color', [0 0.5 0]);
arrow3d([0 0 0], [0 0 L], 'b', 2); % Z轴
text(0, 0, L*1.05, '北极 (N)', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');

% 绘制赤道平面 (O-XY)
patch([-L L L -L], [-L -L L L], [0 0 0 0], 'k', 'FaceAlpha', 0.05, 'EdgeColor', 'none');

% 绘制整个仿真周期内的航天器轨迹
plot3(r_plot(1,:), r_plot(2,:), r_plot(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '轨道轨迹');

%% 5. 动画和数据文本初始化
r_init = r_plot(:, 1);

% 初始化卫星位置的句柄
h_sat = plot3(r_init(1), r_init(2), r_init(3), 'pentagram', ...
    'MarkerSize', 15, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', '卫星');
% 径向矢量线句柄
h_line = line([0 r_init(1)], [0 r_init(2)], [0 r_init(3)], 'Color', 'm', 'LineWidth', 1.5, 'DisplayName', '径向矢量 r'); 
% 卫星标签句柄
h_text = text(r_init(1), r_init(2), r_init(3)+500, ' 卫星 (Satellite)', 'FontWeight', 'bold');

% 数据文本框句柄 (位置基于 max_extent)
text_x = -max_extent * 0.95;
text_y = max_extent * 0.95;
text_z = max_extent * 0.7;
h_data_text = text(text_x, text_y, text_z, '仿真数据...', 'Interpreter', 'none', 'FontSize', 10, 'EdgeColor', 'k', 'BackgroundColor', [1, 1, 1, 0.8], 'HorizontalAlignment', 'left');

%% 6. 动画循环
for k = 1:N
    r_current = r_plot(:, k);
    v_current = v_plot(:, k);
    t_current = t(k);

    % 4. 更新图表元素
    set(h_sat, 'XData', r_current(1), 'YData', r_current(2), 'ZData', r_current(3));
    set(h_line, 'XData', [0 r_current(1)], 'YData', [0 r_current(2)], 'ZData', [0 r_current(3)], 'Color', 'm', 'LineWidth', 1.5);
    set(h_text, 'Position', [r_current(1), r_current(2), r_current(3)+500]);
    
    % 更新状态矢量显示
    coe_string = sprintf(['轨道仿真数据: \n', ...
        '仿真时间 t = %.2f s (T=%.1f s)\n', ...
        '状态矢量 (r, v): \n', ...
        'r = [ %.1f; %.1f; %.1f ] km \n', ...
        'v = [ %.3f; %.3f; %.3f ] km/s'], ...
        t_current, T, ...
        r_current(1), r_current(2), r_current(3), ...
        v_current(1), v_current(2), v_current(3));
    set(h_data_text, 'String', coe_string);

    % 暂停以模拟实时播放 (基于仿真时间差)
    if k < N
        try
            pause(t(k+1) - t(k));
        catch
            pause(1/30);
        end
    end
end
%% 辅助函数
% 绘制带箭头的线 
function arrow3d(p1, p2, color, width)
    p1 = p1(:); p2 = p2(:);
    quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3), ...
        0, 'Color', color, 'LineWidth', width, 'MaxHeadSize', 0.5);
end