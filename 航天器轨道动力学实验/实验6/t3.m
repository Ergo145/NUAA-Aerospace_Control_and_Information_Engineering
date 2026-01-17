%% Task3_Formation_Flying.m
% 实验内容 (3): 600m空间圆编队设计，求解伴随星轨道根数及绘图

clc; clear; close all;

% 1. 物理常数
mu = 3.986004418e14;

% 2. 主星 (Chief) 初始状态
rc0 = [-6311227.13644808; -1112839.6255322; 3700000];
vc0 = [1274.45143292937; -7227.77323794544; 2.2471043881515e-13];

% 主星轨道参数
rc_mag = norm(rc0);
vc_mag = norm(vc0);
ac = -mu / (2*(vc_mag^2/2 - mu/rc_mag));
n = sqrt(mu/ac^3); % 平均角速度
T = 2*pi/n;

fprintf('主星平均角速度 n = %.5e rad/s\n', n);

% 3. 伴随星 (Deputy) 相对运动初始条件设计
% 依据 PDF Page 134-135, "空间圆编队"
d = 600; % 编队半径 600m [cite: 8]
lambda = 0; % 初始相位，设为0

% 计算相对状态 (在 LVLH / Hill 坐标系下)
% 依据 Eq 8.36, 8.37 以及 Eq 8.26 (环绕飞行条件) 和 Eq 8.29 (空间圆平面约束)
dx0 = (d/2) * cos(lambda);                 % Eq 8.36
ddx0 = -(n*d/2) * sin(lambda);             % Eq 8.37
dy0 = 2 * ddx0 / n;                        % Eq 8.26 (由环绕条件推导: dy0 = 2*ddx0/n)
ddy0 = -2 * n * dx0;                       % Eq 8.26 (由环绕条件推导: ddy0 = -2n*dx0)
dz0 = sqrt(3) * dx0;                       % Eq 8.29 (空间圆平面约束: dz = sqrt(3)*dx)
ddz0 = sqrt(3) * ddx0;                     % 对 Eq 8.29 求导

dr_hill = [dx0; dy0; dz0];     % 相对位置
dv_hill = [ddx0; ddy0; ddz0];  % 相对速度

fprintf('相对状态 (LVLH):\n r = [%.2f, %.2f, %.2f] m\n v = [%.4f, %.4f, %.4f] m/s\n', ...
    dr_hill, dv_hill);

% 4. 坐标转换: 相对状态(Hill) -> 绝对状态(Inertial) [cite: 9]
% 计算转换矩阵 Q = [i_r, i_theta, i_h]
r_hat = rc0 / norm(rc0);               % 径向
h_vec = cross(rc0, vc0);
h_hat = h_vec / norm(h_vec);           % 法向 (Cross-track)
theta_hat = cross(h_hat, r_hat);       % 切向 (Along-track)

Q = [r_hat, theta_hat, h_hat]; % 旋转矩阵 (LVLH -> Inertial)

% 伴随星惯性位置
rd0 = rc0 + Q * dr_hill;

% 伴随星惯性速度 (考虑牵连速度)
% v_dep = v_chief + Q*v_rel + w x r_rel (此处简化近似或使用完整公式)
% 在圆轨道假设下 w = n * h_hat
omega_vec = n * h_hat; 
r_rel_inertial = Q * dr_hill;
v_rel_inertial = Q * dv_hill;
vd0 = vc0 + v_rel_inertial + cross(omega_vec, r_rel_inertial);

% 5. 求解伴随星轨道根数 [cite: 11]
[ad, ed, id, Omegad, omegad, f0d] = rv2coe(rd0, vd0, mu);
fprintf('\n========= 伴随星 (Deputy) 六个轨道根数 =========\n');
fprintf('1. 半长轴 (a):        %.4f km\n', ad/1000);
fprintf('2. 偏心率 (e):        %.8f\n', ed);
fprintf('3. 轨道倾角 (i):      %.4f deg\n', rad2deg(id));
fprintf('4. 升交点赤经 (RAAN): %.4f deg\n', rad2deg(Omegad));
fprintf('5. 近地点幅角 (omega):%.4f deg\n', rad2deg(omegad));
fprintf('6. 真近点角 (f):      %.4f deg\n', rad2deg(f0d));
fprintf('================================================\n');

% 6. 仿真与绘图
tspan = [0, T];
options = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);

% 积分主星
[t, Yc] = ode45(@(t,y) TwoBodyODE(t, y, mu), tspan, [rc0; vc0], options);
% 积分伴随星
[~, Yd] = ode45(@(t,y) TwoBodyODE(t, y, mu), tspan, [rd0; vd0], options);

% 计算相对轨迹 (用于验证空间圆)
dr_history = zeros(length(t), 3);
for k = 1:length(t)
    rc = Yc(k, 1:3)'; vc = Yc(k, 4:6)';
    rd = Yd(k, 1:3)';
    
    % 动态计算当前时刻的旋转矩阵
    rh = rc/norm(rc); hh = cross(rc,vc)/norm(cross(rc,vc)); th = cross(hh, rh);
    Q_now = [rh, th, hh];
    
    % 惯性系差值 -> Hill系
    dr_inertial = rd - rc;
    dr_history(k, :) = (Q_now' * dr_inertial)';
end

% --- 绘图 1: 绝对运动轨迹 ---
figure('Color', 'w');
plot3(Yc(:,1), Yc(:,2), Yc(:,3), 'b-', 'LineWidth', 1); hold on;
plot3(Yd(:,1), Yd(:,2), Yd(:,3), 'r--', 'LineWidth', 1);
title('Task 3: 双星惯性系绝对运动轨迹');
legend('主星', '伴随星'); axis equal; grid on; view(3);

% --- 绘图 2: 相对运动轨迹 (验证空间圆) ---
figure('Color', 'w');
plot3(dr_history(:,1), dr_history(:,2), dr_history(:,3), 'k-', 'LineWidth', 2);
grid on; axis equal;
xlabel('Radial (x) [m]'); ylabel('Along-Track (y) [m]'); zlabel('Cross-Track (z) [m]');
title('Task 3: 伴随星相对运动轨迹 (Hill坐标系)');
view([1, 1, 1]); 

% --- 内部函数 ---
function [a, e, i, Omega, omega, f] = rv2coe(r, v, mu)
    r_mag = norm(r); v_mag = norm(v); h = cross(r, v); h_mag = norm(h);
    n_vec = cross([0;0;1], h); n_mag = norm(n_vec);
    vec_e = ((v_mag^2 - mu/r_mag)*r - dot(r,v)*v)/mu; e = norm(vec_e);
    energy = v_mag^2/2 - mu/r_mag; a = -mu / (2*energy);
    i = acos(h(3)/h_mag);
    Omega = acos(n_vec(1)/n_mag); if n_vec(2)<0, Omega=2*pi-Omega; end
    omega = acos(dot(n_vec, vec_e)/(n_mag*e)); if vec_e(3)<0, omega=2*pi-omega; end
    f = acos(dot(vec_e, r)/(e*r_mag)); if dot(r, v)<0, f=2*pi-f; end
end
function dydt = TwoBodyODE(~, y, mu)
    r = y(1:3); v = y(4:6);
    dydt = [v; -mu/norm(r)^3 * r];
end