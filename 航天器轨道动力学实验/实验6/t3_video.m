%% Task3_Final_Solution.m
% 实验内容(3) 完整解决方案
% 功能：
% 1. 计算伴随星轨道根数 (输出到命令行)
% 2. 绘制静态三维图：绝对运动轨迹、相对运动轨迹 (满足实验报告要求)
% 3. 生成动态演示视频 (满足演示需求)

clc; clear; close all;

%% ================= 1. 初始化与轨道设计 =================
% 1.1 物理常数
mu = 3.986004418e14; % 地球引力常数 m^3/s^2
Re = 6378.137e3;     % 地球半径 (用于绘图)

% 1.2 主星 (Chief) 初始状态 [cite: 176]
rc0 = [-6311227.13644808; -1112839.6255322; 3700000];
vc0 = [1274.45143292937; -7227.77323794544; 2.2471043881515e-13];

% 1.3 计算主星轨道参数
rc_mag = norm(rc0);
vc_mag = norm(vc0);
ac = -mu / (2*(vc_mag^2/2 - mu/rc_mag));
n = sqrt(mu/ac^3);            % 平均角速度
T = 2*pi/n;                   % 周期

% 1.4 伴随星 (Deputy) 相对运动设计 (600m 空间圆) [cite: 8]
d = 600;        % 编队半径 600m
lambda = 0;     % 初始相位

% C-W方程初始条件 (Hill坐标系) [cite: 167, 168, 126, 139]
dx0 = (d/2) * cos(lambda);
ddx0 = -(n*d/2) * sin(lambda);
dy0 = 2 * ddx0 / n;          % 环绕条件
ddy0 = -2 * n * dx0;         % 环绕条件
dz0 = sqrt(3) * dx0;         % 空间圆平面约束
ddz0 = sqrt(3) * ddx0;

dr_hill = [dx0; dy0; dz0];
dv_hill = [ddx0; ddy0; ddz0];

% 1.5 坐标转换 (Hill -> Inertial)
r_hat = rc0 / norm(rc0);
h_vec = cross(rc0, vc0);
h_hat = h_vec / norm(h_vec);
theta_hat = cross(h_hat, r_hat);
Q_mat = [r_hat, theta_hat, h_hat]; % 旋转矩阵

% 伴随星惯性系状态 [cite: 9]
rd0 = rc0 + Q_mat * dr_hill;
omega_vec = n * h_hat;
vd0 = vc0 + Q_mat * dv_hill + cross(omega_vec, Q_mat * dr_hill);

% 1.6 输出伴随星轨道根数 [cite: 11]
[ad, ed, id, Omegad, omegad, f0d] = rv2coe(rd0, vd0, mu);
fprintf('\n=== 伴随星六个轨道根数 ===\n');
fprintf('a  (半长轴):   %.4f km\n', ad/1000);
fprintf('e  (偏心率):   %.8f\n', ed);
fprintf('i  (倾角):     %.4f deg\n', rad2deg(id));
fprintf('Om (升交点):   %.4f deg\n', rad2deg(Omegad));
fprintf('om (近地点):   %.4f deg\n', rad2deg(omegad));
fprintf('f  (真近点角): %.4f deg\n', rad2deg(f0d));
fprintf('==========================\n');

%% ================= 2. 轨道积分与数据处理 =================
t_vec = linspace(0, 5*T, 500); % 积分一个周期，取500个点
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);

[~, Yc] = ode45(@(t,y) TwoBodyODE(t, y, mu), t_vec, [rc0; vc0], options);
[~, Yd] = ode45(@(t,y) TwoBodyODE(t, y, mu), t_vec, [rd0; vd0], options);

% 计算历史相对位置 (转换回 Hill 坐标系用于绘图)
rel_pos_history = zeros(length(t_vec), 3);
for k = 1:length(t_vec)
    rc_k = Yc(k, 1:3)'; vc_k = Yc(k, 4:6)';
    rd_k = Yd(k, 1:3)';
    
    % 每一时刻的旋转矩阵
    rh = rc_k / norm(rc_k);
    hh = cross(rc_k, vc_k); hh = hh / norm(hh);
    th = cross(hh, rh);
    Q_now = [rh, th, hh];
    
    % 惯性系差值 -> 投影到 Hill 系
    rel_pos_history(k, :) = (Q_now' * (rd_k - rc_k))'; 
end

%% ================= 3. 绘制静态图 (实验报告截图用) =================

% --- 图1：两颗卫星的三维轨迹图 (惯性系) ---
figure('Name', 'Task3_Inertial_Orbit', 'Color', 'w');
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-', 'LineWidth', 1.5); hold on;
plot3(Yd(:,1), Yd(:,2), Yd(:,3), 'r--', 'LineWidth', 1.5); % 伴随星
% 画地心
[sx, sy, sz] = sphere(20);
surf(sx*Re, sy*Re, sz*Re, 'FaceColor', [0.9 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal; grid on; box on;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
legend('主星绝对轨道', '伴随星绝对轨道', '地球');
title('两颗卫星的惯性系三维轨迹图');
view(3);

% --- 图2：两颗卫星相对运动的三维轨迹图 (Hill系) ---
figure('Name', 'Task3_Relative_Orbit', 'Color', 'w');
plot3(rel_pos_history(:,1), rel_pos_history(:,2), rel_pos_history(:,3), 'k-', 'LineWidth', 2); hold on;
plot3(0, 0, 0, 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 8); % 主星位置
grid on; axis equal; box on;
xlabel('Radial / R (m)'); ylabel('Along-Track / V (m)'); zlabel('Cross-Track / N (m)');
legend('伴随星相对轨迹', '主星');
title('伴随星相对运动三维轨迹 (空间圆)');
view(3);

%% ================= 4. 生成演示视频 =================
fprintf('正在生成视频...\n');
video_name = 'Satellite_Formation_Demo.avi';
v_writer = VideoWriter(video_name);
v_writer.FrameRate = 30;
open(v_writer);

h_vid_fig = figure('Name', 'Video_Animation', 'Color', 'w', 'Position', [100, 100, 1000, 500]);

% 左子图：惯性系
subplot(1,2,1);
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-'); hold on;
% h_c_pt = plot3(Yc(1,1), Yc(1,2), Yc(1,3), 'bo', 'MarkerFaceColor', 'b');
h_c_pt = plot3(Yc(1,1), Yc(1,2), Yc(1,3), 'bo', ...
    'MarkerFaceColor', [1.0 0.8 0.8], ...  % 浅粉色RGB值
    'MarkerEdgeColor', [0.9 0.6 0.6], ...  % 边框稍深一点的粉色
    'MarkerSize', 8);
h_d_pt = plot3(Yd(1,1), Yd(1,2), Yd(1,3), 'r.', 'MarkerSize', 8);
surf(sx*Re, sy*Re, sz*Re, 'FaceColor', [0.9 0.9 1], 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal; grid on; view(3);
title('绝对运动 (惯性系)');
xlim([-1.5*ac, 1.5*ac]); ylim([-1.5*ac, 1.5*ac]); zlim([-1.5*ac, 1.5*ac]);

% 右子图：相对系
subplot(1,2,2);
plot3(rel_pos_history(:,1), rel_pos_history(:,2), rel_pos_history(:,3), 'k--'); hold on;
plot3(0,0,0, 'bo', 'MarkerFaceColor', 'b');
h_d_rel = plot3(rel_pos_history(1,1), rel_pos_history(1,2), rel_pos_history(1,3), ...
    'ro', 'MarkerFaceColor', 'r');
axis equal; grid on; view(3);
title('相对运动 (Hill系)');
max_r = max(abs(rel_pos_history(:))) * 1.5;
xlim([-max_r, max_r]); ylim([-max_r, max_r]); zlim([-max_r, max_r]);

% 动画循环
for k = 1:5:length(t_vec) % 每隔5个点画一帧，加快生成速度
    % 更新数据
    set(h_c_pt, 'XData', Yc(k,1), 'YData', Yc(k,2), 'ZData', Yc(k,3));
    set(h_d_pt, 'XData', Yd(k,1), 'YData', Yd(k,2), 'ZData', Yd(k,3));
    set(h_d_rel, 'XData', rel_pos_history(k,1), ...
                 'YData', rel_pos_history(k,2), ...
                 'ZData', rel_pos_history(k,3));
             
    drawnow;
    frame = getframe(h_vid_fig);
    writeVideo(v_writer, frame);
end
close(v_writer);
fprintf('视频生成完毕: %s\n', video_name);

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