clear; clc; close all;

% 加载任务二的最终结果
load('task2_result.mat'); 

% --- 1. 计算轨道根数 ---
coe = iod_tools.rv2coe(r2_iter, v2_iter, mu);
a = coe(1); e = coe(2); inc = coe(3); 
RAAN = coe(4); w = coe(5); nu = coe(6);

fprintf('\n================ 最终轨道根数 ================\n');
fprintf('半长轴 (a):     %.4f km\n', abs(a));
fprintf('偏心率 (e):     %.6f\n', e);
fprintf('倾角 (i):       %.4f deg\n', inc);

fprintf('升交点赤经 (Ω): %.4f deg\n', RAAN);
fprintf('近地点幅角 (ω): %.4f deg\n', w);
fprintf('真近点角 (ν):   %.4f deg\n', nu);

% --- 2. 准备绘图 ---
figure('Color', 'w', 'Name', '初始轨道确定结果');
hold on; grid on; axis equal;
view(3);

% 坐标轴与标题
xlabel('X (km)', 'FontWeight', 'bold'); 
ylabel('Y (km)', 'FontWeight', 'bold'); 
zlabel('Z (km)', 'FontWeight', 'bold');
title(['轨道几何展示 (e = ' num2str(e, '%.4f') ')'], 'FontSize', 12);

% --- 3. 绘制地球 ---
[x_e, y_e, z_e] = sphere(50);
surf(x_e*Re, y_e*Re, z_e*Re, 'FaceColor', [0.4, 0.6, 1.0], ...
    'EdgeColor', 'none', 'FaceAlpha', 0.6); % 半透明蓝色地球

% --- 4. 绘制轨道轨迹 (修复核心逻辑) ---
if e < 1 && a > 0
    % --- 椭圆轨道 (封闭) ---
    fprintf('>> 检测到椭圆轨道，绘制完整周期...\n');
    T_period = 2*pi * sqrt(a^3/mu);
    t_span = linspace(0, T_period, 500);
    orb_color = 'g'; % 绿色
else
    % --- 双曲线轨道 (开放) ---
    fprintf('>> 检测到双曲线/逃逸轨道 (a < 0)，绘制近地点附近轨迹...\n');
    t_span = linspace(-4000, 4000, 500); 
    orb_color = 'r'; % 红色表示逃逸
end

traj_pos = zeros(3, length(t_span));
for i = 1:length(t_span)
    [f_p, g_p] = iod_tools.universal_fg_exact(r2_iter, v2_iter, t_span(i), mu);
    traj_pos(:,i) = f_p * r2_iter + g_p * v2_iter;
end

plot3(traj_pos(1,:), traj_pos(2,:), traj_pos(3,:), ...
    'Color', orb_color, 'LineWidth', 2);

% --- 5. 绘制观测几何 ---
plot3([0 R(1,2)], [0 R(2,2)], [0 R(3,2)], 'k--', 'LineWidth', 1);
text(R(1,2)*1.1, R(2,2)*1.1, R(3,2)*1.1, '观测站', 'FontSize', 10);

% 卫星位置点
h1 = plot3(r1_n(1), r1_n(2), r1_n(3), 'mo', 'MarkerSize', 6, 'MarkerFaceColor', 'm');
h2 = plot3(r2_n(1), r2_n(2), r2_n(3), 'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
h3 = plot3(r3_n(1), r3_n(2), r3_n(3), 'co', 'MarkerSize', 6, 'MarkerFaceColor', 'c');

if e < 1
    orb_label = '椭圆轨道';
else
    orb_label = '双曲线轨道';
end
legend([h1, h2, h3], {'t1 位置', 't2 位置 (解)', 't3 位置'}, 'Location', 'best');

% 调整视角以便看清轨迹
camlight; lighting gouraud;
axis vis3d;
hold off;