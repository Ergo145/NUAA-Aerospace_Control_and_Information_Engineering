%% Task2_Orbital_Elements.m
% 实验内容 (2): 利用位置速度与轨道根数转换关系，求解一个周期的位置速度并画图

clc; clear; close all;

% 1. 物理常数与初始状态
mu = 3.986004418e14;
r0_vec = [-6311227.13644808; -1112839.6255322; 3700000];
v0_vec = [1274.45143292937; -7227.77323794544; 2.2471043881515e-13];

% 2. 初始状态 -> 轨道根数 (COE)
[a, e, i, Omega, omega, f0] = rv2coe(r0_vec, v0_vec, mu);

% 计算周期和平均角速度
T = 2*pi * sqrt(a^3/mu);
n = sqrt(mu/a^3);

% 计算初始平近点角 M0
E0 = 2 * atan(sqrt((1-e)/(1+e)) * tan(f0/2)); % 偏近点角
M0 = E0 - e * sin(E0);

% 3. 在一个周期内通过根数推算位置
dt = 10; % 时间步长 (s)
time_vec = 0:dt:T;
pos_history = zeros(length(time_vec), 3);

for k = 1:length(time_vec)
    t = time_vec(k);
    
    % 当前平近点角
    M = M0 + n * t;
    
    % 求解开普勒方程 M = E - e*sin(E) 得到 E
    E = solveKepler(M, e);
    
    % 计算真近点角 f
    f = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2));
    
    % 轨道根数 -> 状态向量
    [r_curr, ~] = coe2rv(a, e, i, Omega, omega, f, mu);
    pos_history(k, :) = r_curr';
end

% 4. 绘图
figure('Color', 'w');
plot3(pos_history(:,1), pos_history(:,2), pos_history(:,3), 'g--', 'LineWidth', 1.5); hold on;
plot3(0, 0, 0, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
title('Task 2: 基于轨道根数推演的主星轨迹');
legend('卫星轨迹 (根数法)', '地心');
view(3);

% --- 辅助函数: RV 转 COE ---
function [a, e, i, Omega, omega, f] = rv2coe(r, v, mu)
    r_mag = norm(r); v_mag = norm(v);
    h = cross(r, v); h_mag = norm(h);
    n_vec = cross([0;0;1], h); n_mag = norm(n_vec);
    
    vec_e = ((v_mag^2 - mu/r_mag)*r - dot(r,v)*v)/mu;
    e = norm(vec_e);
    
    energy = v_mag^2/2 - mu/r_mag;
    a = -mu / (2*energy);
    
    i = acos(h(3)/h_mag);
    Omega = acos(n_vec(1)/n_mag);
    if n_vec(2) < 0, Omega = 2*pi - Omega; end
    
    omega = acos(dot(n_vec, vec_e)/(n_mag*e));
    if vec_e(3) < 0, omega = 2*pi - omega; end
    
    f = acos(dot(vec_e, r)/(e*r_mag));
    if dot(r, v) < 0, f = 2*pi - f; end
end

% --- 辅助函数: COE 转 RV ---
function [r_vec, v_vec] = coe2rv(a, e, i, Omega, omega, f, mu)
    p = a * (1 - e^2);
    r_mag = p / (1 + e*cos(f));
    
    r_pqw = [r_mag*cos(f); r_mag*sin(f); 0];
    v_pqw = sqrt(mu/p) * [-sin(f); e+cos(f); 0];
    
    % 旋转矩阵 PQW -> IJK
    R3_W = [cos(-Omega) sin(-Omega) 0; -sin(-Omega) cos(-Omega) 0; 0 0 1];
    R1_i = [1 0 0; 0 cos(-i) sin(-i); 0 -sin(-i) cos(-i)];
    R3_w = [cos(-omega) sin(-omega) 0; -sin(-omega) cos(-omega) 0; 0 0 1];
    
    rot_mat = R3_W * R1_i * R3_w;
    
    r_vec = rot_mat * r_pqw;
    v_vec = rot_mat * v_pqw;
end

% --- 辅助函数: 求解开普勒方程 (牛顿迭代) ---
function E = solveKepler(M, e)
    E = M; 
    tol = 1e-8;
    for k = 1:100
        f_val = E - e*sin(E) - M;
        df_val = 1 - e*cos(E);
        E_new = E - f_val/df_val;
        if abs(E_new - E) < tol
            E = E_new; return;
        end
        E = E_new;
    end
end