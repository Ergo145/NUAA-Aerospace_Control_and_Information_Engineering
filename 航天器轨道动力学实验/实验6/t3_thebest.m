clc; clear; close all;

%% ================= 1. 初始化与轨道设计 =================
mu = 3.986004418e14; % 地球引力常数
Re = 6378.137e3;     % 地球半径

% 1.1 主星 (Chief) 初始状态
rc0 = [-6311227.13644808; -1112839.6255322; 3700000];
vc0 = [1274.45143292937; -7227.77323794544; 2.2471043881515e-13];

% 1.2 计算主星轨道参数
rc_mag = norm(rc0); vc_mag = norm(vc0);
ac = -mu / (2*(vc_mag^2/2 - mu/rc_mag));
n = sqrt(mu/ac^3); 
Tc = 2*pi/n; % 主星周期

% 1.3 伴随星 (Deputy) 相对运动设计 (600m 空间圆)
d = 600; lambda = 0;
dx0 = (d/2) * cos(lambda);      ddx0 = -(n*d/2) * sin(lambda);
dy0 = 2 * ddx0 / n;             ddy0 = -2 * n * dx0;
dz0 = sqrt(3) * dx0;            ddz0 = sqrt(3) * ddx0;
dr_hill = [dx0; dy0; dz0];      dv_hill = [ddx0; ddy0; ddz0];

% 1.4 坐标转换 (Hill -> Inertial)
r_hat = rc0 / norm(rc0);
h_vec = cross(rc0, vc0); h_hat = h_vec / norm(h_vec);
theta_hat = cross(h_hat, r_hat);
Q_mat = [r_hat, theta_hat, h_hat];

rd0 = rc0 + Q_mat * dr_hill;
omega_vec = n * h_hat;
vd0 = vc0 + Q_mat * dv_hill + cross(omega_vec, Q_mat * dr_hill);

% 1.5 计算伴随星轨道根数
[ad, ed, id, Omegad, omegad, f0d] = rv2coe(rd0, vd0, mu);
Td = 2*pi * sqrt(ad^3/mu); % 伴星周期

% 1.6 输出：伴随星六个轨道根数
fprintf('\n=== 伴随星六个轨道根数 ===\n');
fprintf('a  (半长轴):   %.4f km\n', ad/1000);
fprintf('e  (偏心率):   %.8f\n', ed);
fprintf('i  (倾角):     %.4f deg\n', rad2deg(id));
fprintf('Om (升交点):   %.4f deg\n', rad2deg(Omegad));
fprintf('om (近地点):   %.4f deg\n', rad2deg(omegad));
fprintf('f  (真近点角): %.4f deg\n', rad2deg(f0d));
fprintf('==========================\n');

% 1.7 输出：轨道周期
fprintf('\n=== 轨道周期 ===\n');
fprintf('主星周期 (Chief Period):  %.4f s\n', Tc);
fprintf('伴星周期 (Deputy Period): %.4f s\n', Td);
fprintf('========================\n');

%% ================= 2. 轨道积分 =================
t_vec = linspace(0, Tc, 600); % 积分一个周期
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[~, Yc] = ode45(@(t,y) TwoBodyODE(t, y, mu), t_vec, [rc0; vc0], options);
[~, Yd] = ode45(@(t,y) TwoBodyODE(t, y, mu), t_vec, [rd0; vd0], options);

% 预计算相对位置 (Hill系)
rel_pos_history = zeros(length(t_vec), 3);
for k = 1:length(t_vec)
    rc_k = Yc(k, 1:3)'; vc_k = Yc(k, 4:6)'; rd_k = Yd(k, 1:3)';
    rh = rc_k / norm(rc_k); hh = cross(rc_k, vc_k); hh = hh / norm(hh); th = cross(hh, rh);
    Q_now = [rh, th, hh];
    rel_pos_history(k, :) = (Q_now' * (rd_k - rc_k))'; 
end

%% ================= 3. 绘制静态图 (实验报告用) =================
figure('Name', 'Static_Orbits', 'Color', 'w', 'Position', [100, 100, 1000, 400]);
% 左图：惯性系
subplot(1,2,1);
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-'); hold on;
plot3(Yd(:,1), Yd(:,2), Yd(:,3), 'r--');
[sx, sy, sz] = sphere(20);
surf(sx*Re, sy*Re, sz*Re, 'FaceColor', [0.9 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal; grid on; view(3); title('惯性系绝对轨迹');
% 右图：相对系
subplot(1,2,2);
plot3(rel_pos_history(:,1), rel_pos_history(:,2), rel_pos_history(:,3), 'k-'); hold on;
plot3(0,0,0, 'bo', 'MarkerFaceColor', 'b');
axis equal; grid on; view(3); title('Hill系相对轨迹');

%% ================= 4. 动态效果 =================
video_name = 'Satellite_Final_Demo.avi';
v_writer = VideoWriter(video_name);
v_writer.FrameRate = 30;
open(v_writer);

h_vid_fig = figure('Name', 'Video_Animation', 'Color', 'w', 'Position', [50, 100, 1500, 500]);

% --- 子图 1：宏观地球 (Global) ---
subplot(1,3,1);
surf(sx*Re, sy*Re, sz*Re, 'FaceColor', [0.8 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5); hold on;
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-', 'LineWidth', 1); % 完整轨道
h_glob_c = plot3(Yc(1,1), Yc(1,2), Yc(1,3), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 5);
axis equal; grid on; view(3);
title('1. 宏观视角 (Global Earth)');
xlabel('X'); ylabel('Y'); zlabel('Z');
axis([-2*Re 2*Re -2*Re 2*Re -2*Re 2*Re]);

% --- 子图 2：惯性系跟随 (Inertial Zoom) ---
ax2 = subplot(1,3,2);
hold on; grid on; axis equal; view(3); 
% 预画蓝色轨道线
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-', 'LineWidth', 0.5); 
% 初始化点
h_loc_c = plot3(0,0,0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8); 
h_loc_d = plot3(0,0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8); 
title('2. 惯性系伴飞');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

% --- 子图 3：相对运动 (Hill Frame) ---
subplot(1,3,3);
plot3(rel_pos_history(:,1), rel_pos_history(:,2), rel_pos_history(:,3), 'k--', 'Color', [0.8 0.8 0.8]); hold on;
plot3(0,0,0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8);
h_d_rel = plot3(0,0,0, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
h_trail_rel = plot3(0,0,0, 'r-', 'LineWidth', 2); % 
axis equal; grid on; view(3);
title('3. 相对运动 (Hill Frame)');
max_r = 1000; 
xlim([-max_r, max_r]); ylim([-max_r, max_r]); zlim([-max_r, max_r]);

% --- 动画循环 ---
view_radius = 1200; 
step = 2; 

for k = 1:step:length(t_vec)
    cx = Yc(k,1); cy = Yc(k,2); cz = Yc(k,3);
    dx = Yd(k,1); dy = Yd(k,2); dz = Yd(k,3);
    
    % Update Global
    set(h_glob_c, 'XData', cx, 'YData', cy, 'ZData', cz);
    
    % Update Inertial Zoom
    set(h_loc_c, 'XData', cx, 'YData', cy, 'ZData', cz);
    set(h_loc_d, 'XData', dx, 'YData', dy, 'ZData', dz);
    % 动态坐标框跟随
    set(ax2, 'XLim', [cx - view_radius, cx + view_radius], ...
             'YLim', [cy - view_radius, cy + view_radius], ...
             'ZLim', [cz - view_radius, cz + view_radius]);
             
    % Update Relative
    set(h_d_rel, 'XData', rel_pos_history(k,1), 'YData', rel_pos_history(k,2), 'ZData', rel_pos_history(k,3));
    set(h_trail_rel, 'XData', rel_pos_history(1:k,1), 'YData', rel_pos_history(1:k,2), 'ZData', rel_pos_history(1:k,3));
    
    drawnow;
    frame = getframe(h_vid_fig);
    writeVideo(v_writer, frame);
end
close(v_writer);

%% ================= 辅助函数 =================
function dydt = TwoBodyODE(~, y, mu)
    r = y(1:3); v = y(4:6);
    dydt = [v; -mu/norm(r)^3 * r];
end

function [a, e, i, Omega, omega, f] = rv2coe(r, v, mu)
    r_mag = norm(r); v_mag = norm(v); h = cross(r, v); h_mag = norm(h);
    n_vec = cross([0;0;1], h); n_mag = norm(n_vec);
    if n_mag == 0, n_vec = [1;0;0]; end
    vec_e = ((v_mag^2 - mu/r_mag)*r - dot(r,v)*v)/mu; e = norm(vec_e);
    energy = v_mag^2/2 - mu/r_mag; a = -mu / (2*energy);
    i = acos(h(3)/h_mag);
    Omega = acos(n_vec(1)/n_mag); if n_vec(2)<0, Omega=2*pi-Omega; end
    omega = acos(dot(n_vec, vec_e)/(n_mag*e)); 
    if isnan(omega), omega=0; end
    if vec_e(3)<0, omega=2*pi-omega; end
    f = acos(dot(vec_e, r)/(e*r_mag)); 
    if isnan(f), f=0; end
    if dot(r, v)<0, f=2*pi-f; end
end