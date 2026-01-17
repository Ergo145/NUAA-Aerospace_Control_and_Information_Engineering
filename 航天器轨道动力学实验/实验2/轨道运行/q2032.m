%% 主程序：航天器轨道根数转换与三维仿真 (最终修正版)
clc; clear; close all;
%% 1. 定义轨道六根数
a = 10000;      % 半长轴 (km)
e = 0.3;        % 偏心率 (0 < e < 1)
i_deg = 45;     % 轨道倾角 (degrees)
Omega_deg = 60; % 升交点赤经 (degrees)
omega_deg = 30; % 近地点幅角 (degrees)
M0_deg = 45;    % 卫星当前的初始平近点角
% 物理常数
mu = 398600;    % 地球引力常数
Re = 6378;      % 地球半径 (km)
%% 2. 准备绘图环境
% ***** 修改点：调整图窗尺寸为 1000x600 (横向拉长) *****
figure('Color', 'w', 'Position', [100, 100, 1000, 600]); 
% *******************************************
hold on; axis equal; grid on;
view(120, 25); % 设置视角
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('航天器轨道三维仿真与运动动画');
% 设置坐标轴范围，以保证动画运行稳定
max_extent = a * (1 + e) * 1.1; 
xlim([-max_extent, max_extent]);
ylim([-max_extent, max_extent]);
zlim([-max_extent, max_extent]);

% 绘制地球
[x_e, y_e, z_e] = sphere(50);
surf(x_e*Re, y_e*Re, z_e*Re, 'FaceColor', [0.9, 0.9, 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.5);
%% 3. 绘制坐标系 (地心惯性坐标系 O-XYZ)
L = a * 1.8; % 坐标轴长度
% X轴 (春分点方向)
arrow3d([0 0 0], [L 0 0], 'r', 2);
text(L*1.05, 0, 0, '春分点 (\gamma)', 'Color', 'r', 'FontSize', 12, 'FontWeight', 'bold');
% Y轴
arrow3d([0 0 0], [0 L 0], [0 0.5 0], 1);
text(0, L*1.05, 0, 'Y', 'Color', [0 0.5 0]);
% Z轴 (北极方向)
arrow3d([0 0 0], [0 0 L], 'b', 2);
text(0, 0, L*1.05, '北极 (N)', 'Color', 'b', 'FontSize', 12, 'FontWeight', 'bold');
% 绘制赤道平面 (O-XY)
patch([-L L L -L], [-L -L L L], [0 0 0 0], 'k', 'FaceAlpha', 0.05, 'EdgeColor', 'none');

%% 4. 绘制完整轨道
theta_range = linspace(0, 360, 500);
orbit_points = zeros(3, length(theta_range));
for k = 1:length(theta_range)
    orbit_points(:, k) = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_range(k));
end
plot3(orbit_points(1,:), orbit_points(2,:), orbit_points(3,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '轨道');

%% 5. 计算并标注关键点 (Points of Interest) - 重新添加交点
% --- 5.1 近地点 (Perigee, theta=0) ---
pos_peri = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, 0);
plot_point(pos_peri, 'ro', ' 近地点 (Perigee)');
line([0 pos_peri(1)], [0 pos_peri(2)], [0 pos_peri(3)], 'Color', 'r', 'LineStyle', '--');
% --- 5.2 远地点 (Apogee, theta=180) ---
pos_apo = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, 180);
plot_point(pos_apo, 'bs', ' 远地点 (Apogee)');
line([0 pos_apo(1)], [0 pos_apo(2)], [0 pos_apo(3)], 'Color', 'b', 'LineStyle', ':');
% --- 5.3 升交点 (Ascending Node, theta=-omega) ---
theta_asc = -omega_deg; 
pos_asc = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_asc);
plot_point(pos_asc, 'kx', ' 升交点 (\Omega)');
line([0 pos_asc(1)], [0 pos_asc(2)], [0 pos_asc(3)], 'Color', 'k', 'LineStyle', '-.', 'LineWidth', 1.5, 'DisplayName', '升交线'); % 升交线
% --- 5.4 降交点 (Descending Node, theta=180-omega) ---
theta_desc = 180 - omega_deg;
pos_desc = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_desc);
plot_point(pos_desc, 'ko', ' 降交点');
line([0 pos_desc(1)], [0 pos_desc(2)], [0 pos_desc(3)], 'Color', 'k', 'LineStyle', ':');
% --- 5.5 标注辅助线：近地点幅角 omega ---
% ***** 修正点：将 LaTeX 语法改为 MATLAB Tex 语法，避免警告 *****
text(0, 0, 0, '  \omega (近地点幅角)', 'Color', 'k', 'HorizontalAlignment', 'center', 'Interpreter', 'tex');
% *******************************************

%% 6. 动画设置和视频保存 
% 计算轨道周期 T (秒) 和平运动角速度 n (rad/s)
T = 2 * pi * sqrt(a^3 / mu); % 轨道周期 (秒)
n = sqrt(mu / a^3);         % 平运动角速度 (rad/s)

% --- 目标：仿真时长 (视频播放时长) 为 30 秒，包含 1 个周期 ---
sim_duration_actual = T; % 实际物理仿真时间设置为一个周期 T
video_play_time = 30;    % 目标视频播放时长 (秒)
FPS = 30;                % 视频帧率 (Frames Per Second)

num_frames = round(video_play_time * FPS); 
time_step = sim_duration_actual / num_frames; 

fprintf('卫星轨道周期 T = %.1f 秒 (%.2f 小时)\n', T, T/3600);

% 初始化视频写入对象
video_file_name = 'Spacecraft_Orbit_30s_Per_Revolution_Full_Nodes.mp4';
v = VideoWriter(video_file_name, 'MPEG-4');
v.FrameRate = FPS;
open(v);

% 初始化卫星位置的句柄 (用于动画更新)
[~, theta_current_init] = kepler_solve(M0_deg, e); 
pos_sat_init = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_current_init);

% 动画元素句柄
h_sat = plot3(pos_sat_init(1), pos_sat_init(2), pos_sat_init(3), 'pentagram', ...
    'MarkerSize', 15, 'MarkerFaceColor', 'y', 'MarkerEdgeColor', 'k', 'DisplayName', '卫星');
% 径向矢量线句柄
h_line = line([0 pos_sat_init(1)], [0 pos_sat_init(2)], [0 pos_sat_init(3)], 'Color', 'm', 'LineWidth', 1.5, 'DisplayName', '径向矢量 r'); 
% 卫星标签句柄
h_text = text(pos_sat_init(1), pos_sat_init(2), pos_sat_init(3)+500, ' 卫星 (Satellite)', 'FontWeight', 'bold');
% 数据文本框句柄 (位置基于 max_extent)
text_x = -max_extent * 0.95;
text_y = max_extent * 0.95;
text_z = max_extent * 0.7;
h_data_text = text(text_x, text_y, text_z, '仿真数据...', 'Interpreter', 'none', 'FontSize', 10, 'EdgeColor', 'k', 'BackgroundColor', [1, 1, 1, 0.8], 'HorizontalAlignment', 'left');

% 动画循环
for k = 0:num_frames
    t = k * time_step; % 当前物理时间
    
    % 1. 计算当前平近点角 M (rad)
    M = deg2rad(M0_deg) + n * t;
    M_deg = rad2deg(M);
    
    % 2. 解开普勒方程 M -> E -> theta
    [~, theta_current] = kepler_solve(M_deg, e); 
    
    % 3. 计算当前位置和速度 (ECI坐标系)
    pos_sat = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_current);
    vel_sat = calc_vel_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_current, mu); 

    % 4. 更新图表元素
    set(h_sat, 'XData', pos_sat(1), 'YData', pos_sat(2), 'ZData', pos_sat(3));
    set(h_line, 'XData', [0 pos_sat(1)], 'YData', [0 pos_sat(2)], 'ZData', [0 pos_sat(3)]);
    set(h_text, 'Position', [pos_sat(1), pos_sat(2), pos_sat(3)+500]);
    
    % 更新状态矢量显示
    coe_string = sprintf(['轨道仿真数据: \n', ...
        '物理时间 t = %.1f s (T=%.1f s)\n', ...
        'M (平近点角) = %.2f deg \n', ...
        'θ (真近点角) = %.2f deg \n\n', ...
        '状态矢量 (r, v): \n', ...
        'r = [ %.1f; %.1f; %.1f ] km \n', ...
        'v = [ %.3f; %.3f; %.3f ] km/s'], ...
        t, T, M_deg, theta_current, ...
        pos_sat(1), pos_sat(2), pos_sat(3), ...
        vel_sat(1), vel_sat(2), vel_sat(3));
    set(h_data_text, 'String', coe_string);

    % 5. 捕获帧并写入视频
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v); % 关闭视频文件
disp(['视频文件已保存到: ', video_file_name]);

%% ================= 子函数区域 (保持不变) =================
% 1. 由 a, e, theta 计算轨道半径 r
function r = calc_r_from_coe(a, e, theta_deg)
    theta = deg2rad(theta_deg);
    r = a * (1 - e^2) / (1 + e * cos(theta));
end
% 2. 轨道根数 -> 空间位置 r_ECI (通过真近点角)
function r_ECI = calc_pos_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_deg)
    i = deg2rad(i_deg);
    Omega = deg2rad(Omega_deg);
    omega = deg2rad(omega_deg);
    theta = deg2rad(theta_deg);
    r = calc_r_from_coe(a, e, theta_deg);
    r_pqw = [r * cos(theta); r * sin(theta); 0];
    % 旋转矩阵 R (PQW -> ECI)
    R = [cos(Omega)*cos(omega)-sin(Omega)*sin(omega)*cos(i), -cos(Omega)*sin(omega)-sin(Omega)*cos(omega)*cos(i),  sin(Omega)*sin(i);
         sin(Omega)*cos(omega)+cos(Omega)*sin(omega)*cos(i), -sin(Omega)*sin(omega)+cos(Omega)*cos(omega)*cos(i), -cos(Omega)*sin(i);
         sin(omega)*sin(i),                                   cos(omega)*sin(i),                                    cos(i)];
    r_ECI = R * r_pqw;
end
% 3. 轨道根数 -> 速度 V_ECI (通过真近点角)
function v_ECI = calc_vel_from_theta(a, e, i_deg, Omega_deg, omega_deg, theta_deg, mu)
    i = deg2rad(i_deg);
    Omega = deg2rad(Omega_deg);
    omega = deg2rad(omega_deg);
    theta = deg2rad(theta_deg);
    h = sqrt(mu * a * (1 - e^2));
    v_pqw = (mu / h) * [-sin(theta); e + cos(theta); 0];
    % 旋转矩阵 R (PQW -> ECI)
    R = [cos(Omega)*cos(omega)-sin(Omega)*sin(omega)*cos(i), -cos(Omega)*sin(omega)-sin(Omega)*cos(omega)*cos(i),  sin(Omega)*sin(i);
         sin(Omega)*cos(omega)+cos(Omega)*sin(omega)*cos(i), -sin(Omega)*sin(omega)+cos(Omega)*cos(omega)*cos(i), -cos(Omega)*sin(i);
         sin(omega)*sin(i),                                   cos(omega)*sin(i),                                    cos(i)];
    v_ECI = R * v_pqw;
end
% 4. 状态矢量 -> 轨道六根数 (rv2coe) (未在动画中使用，保留)
function [a, e, i_deg, Omega_deg, omega_deg, theta_deg] = rv2coe(r, v, mu)
    r_mag = norm(r); 
    v_mag = norm(v); 
    h = cross(r, v);
    h_mag = norm(h);
    i = acos(h(3) / h_mag);
    i_deg = rad2deg(i);
    k_vec = [0; 0; 1];
    n = cross(k_vec, h);
    n_mag = norm(n);
    if n_mag ~= 0
        Omega = acos(n(1) / n_mag);
        if n(2) < 0 
            Omega = 2 * pi - Omega;
        end
    else 
        Omega = 0; 
    end
    Omega_deg = mod(rad2deg(Omega), 360);
    e_vec = (1/mu) * ((v_mag^2 - mu/r_mag) * r - dot(r, v) * v);
    e = norm(e_vec);
    a = mu / (2 * mu/r_mag - v_mag^2);
    if e > 1e-10 
        omega = acos(dot(n, e_vec) / (n_mag * e));
        if e_vec(3) < 0 
            omega = 2 * pi - omega;
        end
    else 
        omega = 0; 
    end
    omega_deg = mod(rad2deg(omega), 360);
    if e > 1e-10 
        theta = acos(dot(e_vec, r) / (e * r_mag));
        if dot(r, v) < 0 
            theta = 2 * pi - theta;
        end
    else 
        theta = acos(dot(n, r) / (n_mag * r_mag));
        if r(3) < 0 
            theta = 2 * pi - theta;
        end
    end
    theta_deg = mod(rad2deg(theta), 360);
end
% 5. 解开普勒方程 (M -> theta)
function [E, theta_deg] = kepler_solve(M_deg, e)
    M = mod(deg2rad(M_deg), 2*pi); % 确保 M 在 [0, 2pi) 范围内
    E = M; 
    ratio = 1; tol = 1e-8;
    while abs(ratio) > tol
        f = E - e*sin(E) - M;
        df = 1 - e*cos(E);
        ratio = f/df;
        E = E - ratio; 
    end
    % 使用 atan2 确保 theta 范围正确
    theta = 2 * atan2(sqrt(1+e)*sin(E/2), sqrt(1-e)*cos(E/2));
    theta_deg = mod(rad2deg(theta), 360);
end
% 6. 辅助绘点函数
function plot_point(pos, style, label)
    plot3(pos(1), pos(2), pos(3), style, 'MarkerSize', 8, 'LineWidth', 2);
    text(pos(1), pos(2), pos(3), label, 'FontSize', 10, 'BackgroundColor', 'w', 'Margin', 1);
end
% 7. 绘制带箭头的线
function arrow3d(p1, p2, color, width)
    % quiver3 的正确参数应为：起点坐标 X0, Y0, Z0 和 向量分量 U, V, W
    quiver3(p1(1), p1(2), p1(3), p2(1)-p1(1), p2(2)-p1(2), p2(3)-p1(3), ...
        0, 'Color', color, 'LineWidth', width, 'MaxHeadSize', 0.5);
end