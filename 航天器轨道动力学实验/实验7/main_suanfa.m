% -------------------------------------------------------------------------
% 实验名称：近地小行星探测轨道设计 (基于文档第11章原理修正版)
% 修正依据：PDF文档中 Mathematica 代码逻辑 
% -------------------------------------------------------------------------
clc; clear; close all;

%% 0. 智能优化算法 (寻找可行解)
fprintf('正在启动增强型 PSO 算法寻找满足三脉冲约束的轨道，请耐心等待...\n');

% --- 优化参数设置 ---
% [LaunchDate(天), TransferOut(天), StayTime(天), TransferBack(天)]
% 扩大时间窗口以寻找最佳相位角
lb = [0,    100,  10,  100];    
ub = [1000, 450,  90,  450];   

n_particles = 150;      % 增加粒子数应对更难的收敛
n_iterations = 120;     
w_max = 0.9; w_min = 0.4; 
c1 = 2.0; c2 = 2.0;    

% 初始化
vars = 4;
pos = repmat(lb, n_particles, 1) + rand(n_particles, vars) .* repmat(ub-lb, n_particles, 1);
vel = zeros(n_particles, vars);
pbest_pos = pos;
pbest_val = inf(n_particles, 1);
gbest_pos = pos(1,:);
gbest_val = inf;

% 物理常数 (优化内部使用)
opt_mu_sun = 1.32712440018e11;
opt_mu_earth = 398600.4418;
opt_R_park = 6378.137 + 200;
opt_R_reentry = 6378.137 + 110;

% 优化主循环
for iter = 1:n_iterations
    w = w_max - (w_max - w_min) * iter / n_iterations; % 线性递减权重
    
    for i = 1:n_particles
        x = pos(i,:);
        x = max(x, lb); x = min(x, ub); % 边界限制
        pos(i,:) = x;
        
        try
            % 1. 时间计算
            t1 = x(1);                  % 地球出发
            t2 = t1 + x(2);             % 小行星到达
            t3 = t2 + x(3);             % 小行星离开
            t4 = t3 + x(4);             % 地球返回
            
            % 2. 状态矢量计算
            [r_e1, v_e1] = get_body_state('Earth', t1, opt_mu_sun);
            [r_a2, v_a2] = get_body_state('Asteroid', t2, opt_mu_sun);
            [r_a3, v_a3] = get_body_state('Asteroid', t3, opt_mu_sun);
            [r_e4, v_e4] = get_body_state('Earth', t4, opt_mu_sun);
            
            % 3. 去程 Lambert
            [v_dep_start, v_dep_end] = lambert_solver(r_e1, r_a2, x(2)*86400, opt_mu_sun);
            
            % --- 脉冲 1: 地球出发 ---
            v_inf_d = norm(v_dep_start - v_e1);
            dv_1 = sqrt(v_inf_d^2 + 2*opt_mu_earth/opt_R_park) - sqrt(opt_mu_earth/opt_R_park);
            
            % --- 脉冲 2: 小行星到达 (交会刹车) [修正点: 对应文档 line 40] ---
            dv_2 = norm(v_dep_end - v_a2);
            
            % 4. 回程 Lambert
            [v_ret_start, v_ret_end] = lambert_solver(r_a3, r_e4, x(4)*86400, opt_mu_sun);
            
            % --- 脉冲 3: 小行星离开 [修正点: 对应文档 line 45] ---
            dv_3 = norm(v_ret_start - v_a3);
            
            % --- 再入速度 ---
            v_inf_a = norm(v_ret_end - v_e4);
            v_re = sqrt(v_inf_a^2 + 2*opt_mu_earth/opt_R_reentry);
            
            % 5. 代价函数 (总 Delta V)
            total_dv = dv_1 + dv_2 + dv_3;
            cost = total_dv;
            
            % 强惩罚项
            if dv_1 > 6.0, cost = cost + 20 * (dv_1 - 6)^2; end       % 发射能力限制
            if total_dv > 10, cost = cost + 100 * (total_dv - 10)^2; end % 总指标 
            if v_re > 14, cost = cost + 500 * (v_re - 14)^2; end      % 再入限制 
            
        catch
            cost = 1e9;
        end
        
        % 更新最优解
        if cost < pbest_val(i)
            pbest_val(i) = cost;
            pbest_pos(i,:) = x;
        end
        if cost < gbest_val
            gbest_val = cost;
            gbest_pos = x;
        end
    end
    
    % 粒子运动更新
    for i = 1:n_particles
        r1 = rand(1, vars); r2 = rand(1, vars);
        vel(i,:) = w*vel(i,:) + c1*r1.*(pbest_pos(i,:) - pos(i,:)) + c2*r2.*(gbest_pos - pos(i,:));
        pos(i,:) = pos(i,:) + vel(i,:);
        % 随机扰动机制防止早熟
        if rand < 0.02, pos(i,:) = lb + rand(1,vars).*(ub-lb); end
    end
    
    if mod(iter, 20) == 0
        fprintf('优化进度: %d/%d, 当前最佳 Total Delta V: %.4f (含惩罚)\n', iter, n_iterations, gbest_val);
    end
end

% 提取优化结果
launch_date   = gbest_pos(1);
transfer_out  = gbest_pos(2);
stay_time     = gbest_pos(3);
transfer_back = gbest_pos(4);

fprintf('\n--------------------------------------\n');
fprintf('优化完成，推荐参数：\n');
fprintf('Launch Date   = %.2f 天\n', launch_date);
fprintf('Transfer Out  = %.2f 天\n', transfer_out);
fprintf('Stay Time     = %.2f 天\n', stay_time);
fprintf('Transfer Back = %.2f 天\n', transfer_back);
fprintf('--------------------------------------\n\n');

%% 1. 物理常数与参数设置
mu_sun = 1.32712440018e11;  
mu_earth = 398600.4418;     
R_parking = 6378.137 + 200; 
R_reentry = 6378.137 + 110; 

%% 2. 计算与评估 
t1 = launch_date;
t2 = t1 + transfer_out;
t3 = t2 + stay_time;
t4 = t3 + transfer_back;

[r_e1, v_e1] = get_body_state('Earth', t1, mu_sun);
[r_a2, v_a2] = get_body_state('Asteroid', t2, mu_sun);
[r_a3, v_a3] = get_body_state('Asteroid', t3, mu_sun);
[r_e4, v_e4] = get_body_state('Earth', t4, mu_sun);

%% 3. 去程分析
dt_out = transfer_out * 86400;
[v_trans1_start, v_trans1_end] = lambert_solver(r_e1, r_a2, dt_out, mu_sun);

% (1) 发射 Delta V
v_inf_dep = norm(v_trans1_start - v_e1);
v_park = sqrt(mu_earth / R_parking);
v_peri = sqrt(v_inf_dep^2 + 2*mu_earth/R_parking);
delta_v_dep = v_peri - v_park;

% (2) 小行星交会 Delta V (刹车) 
delta_v_arr = norm(v_trans1_end - v_a2);

fprintf('--- 去程分析 ---\n');
fprintf('1. 发射 C3 能量:      %.2f km^2/s^2\n', v_inf_dep^2);
fprintf('2. 地球发射 Delta V:  %.4f km/s\n', delta_v_dep);
fprintf('3. 小行星交会 Delta V: %.4f km/s \n', delta_v_arr);

%% 4. 回程分析
dt_back = transfer_back * 86400;
[v_trans2_start, v_trans2_end] = lambert_solver(r_a3, r_e4, dt_back, mu_sun);

% (3) 小行星离开 Delta V 
delta_v_dep_ast = norm(v_trans2_start - v_a3);

% 再入速度
v_inf_arr = norm(v_trans2_end - v_e4);
v_reentry = sqrt(v_inf_arr^2 + 2*mu_earth/R_reentry);

fprintf('\n--- 回程分析 ---\n');
fprintf('4. 小行星离开 Delta V: %.4f km/s\n', delta_v_dep_ast);
fprintf('5. 地球再入速度:      %.4f km/s\n', v_reentry);

%% 5. 总体可行性评估 
% 总脉冲 = 发射 + 交会 + 离开
total_delta_v = delta_v_dep + delta_v_arr + delta_v_dep_ast;

fprintf('\n--- 最终结果 ---\n');
fprintf('总 Delta V : %.4f km/s\n', total_delta_v);

success = true;
if total_delta_v > 10
    fprintf('[警告] 总 Delta V 超过 10 km/s 阈值！\n');
    success = false;
end
if v_reentry > 14
    fprintf('[警告] 再入速度 超过 14 km/s 阈值！\n');
    success = false;
end

if success
    fprintf('结论：轨道方案 **可行** 。\n');
else
    fprintf('结论：轨道方案 **不可行**。\n');
end

%% 6. 绘图仿真
figure('Color', 'w'); hold on; axis equal; grid on;
view(3); xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
% 修改标题为中文
title(['轨道仿真 (总 \DeltaV = ' num2str(total_delta_v, '%.2f') ' km/s)']);

% 1. 修改太阳图例
plot3(0,0,0, 'yo', 'MarkerSize', 12, 'MarkerFaceColor', 'y', 'DisplayName', '太阳');

% 2. 修改轨道名称图例
plot_orbit('Earth', mu_sun, 'b', '地球轨道');
plot_orbit('Asteroid', mu_sun, 'k', '小行星轨道');

% 3. 修改关键点图例
plot3(r_e1(1), r_e1(2), r_e1(3), 'bo', 'MarkerFaceColor','b', 'DisplayName', '地球出发');
plot3(r_a2(1), r_a2(2), r_a2(3), 'k^', 'MarkerFaceColor','k', 'DisplayName', '到达小行星');
plot3(r_a3(1), r_a3(2), r_a3(3), 'ks', 'MarkerFaceColor','w', 'DisplayName', '离开小行星');
plot3(r_e4(1), r_e4(2), r_e4(3), 'bd', 'MarkerFaceColor','c', 'DisplayName', '返回地球');

% 4. 修改转移轨迹图例
plot_transfer(r_e1, v_trans1_start, dt_out, mu_sun, 'r--', '去程转移');
plot_transfer(r_a3, v_trans2_start, dt_back, mu_sun, 'g--', '回程转移');

% 显示图例
legend('show', 'Location', 'best', 'FontName', 'Microsoft YaHei'); % 建议指定中文字体防止乱码

%% 7. 辅助函数库
function plot_orbit(body_name, mu, color_spec, legend_name)
    dt_step = 3600*24*10; 
    period = 365.25 * 3 * 86400; % 画3年
    t_span = 0:dt_step:period;
    points = zeros(length(t_span), 3);
    for k = 1:length(t_span)
        [r, ~] = get_body_state(body_name, t_span(k)/86400, mu);
        points(k, :) = r;
    end
    plot3(points(:,1), points(:,2), points(:,3), 'Color', color_spec, 'DisplayName', legend_name, 'LineWidth', 1);
end

function plot_transfer(r_start, v_start, dt, mu, style, name)
    options = odeset('RelTol', 1e-6);
    [~, state] = ode45(@(t,y) two_body_ode(t,y,mu), [0, dt], [r_start, v_start], options);
    plot3(state(:,1), state(:,2), state(:,3), style, 'LineWidth', 1.5, 'DisplayName', name);
end

function dydt = two_body_ode(~, y, mu)
    r = y(1:3); v = y(4:6);
    dydt = [v; -mu * r / norm(r)^3];
end

function [r, v] = get_body_state(body_name, days, mu)
    if strcmp(body_name, 'Earth')
        coe = [1.496e8, 0.0167, 0, 0, 0, 0]; 
        n = sqrt(mu / coe(1)^3); M = coe(6) + n * (days * 86400);
    else 
        % 模拟 2013 WA44 (来自原始代码)
        coe = [1.0975044 * 1.496e8, 0.0579408, deg2rad(2.29904), ...
               deg2rad(55.98285), deg2rad(177.65979), deg2rad(338.54721)];
        n = sqrt(mu / coe(1)^3); M = coe(6) + n * (days * 86400);
    end
    
    E = M; 
    for k = 1:15, E = M + coe(2)*sin(E); end % Kepler Iteration
    
    a = coe(1); e = coe(2); p = a * (1 - e^2);
    nu = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2));
    r_mag = p / (1 + e*cos(nu));
    r_pqw = [r_mag*cos(nu); r_mag*sin(nu); 0];
    v_pqw = sqrt(mu/p) * [-sin(nu); e+cos(nu); 0];
    
    O = coe(4); w = coe(5); i = coe(3);
    R3_O = [cos(O) sin(O) 0; -sin(O) cos(O) 0; 0 0 1];
    R1_i = [1 0 0; 0 cos(i) sin(i); 0 -sin(i) cos(i)];
    R3_w = [cos(w) sin(w) 0; -sin(w) cos(w) 0; 0 0 1];
    Q_px = (R3_w * R1_i * R3_O)';
    r = (Q_px * r_pqw)'; v = (Q_px * v_pqw)';
end

function [v1, v2] = lambert_solver(r1, r2, dt, mu)
    r1_norm = norm(r1); r2_norm = norm(r2);
    cross_12 = cross(r1, r2);
    theta = acos(dot(r1, r2) / (r1_norm * r2_norm));
    if cross_12(3) < 0, theta = 2*pi - theta; end
    
    A = sin(theta) * sqrt(r1_norm * r2_norm / (1 - cos(theta)));
    z = 0; iter = 0;
    while iter < 100
        [C, S] = stumpff(z);
        y = r1_norm + r2_norm + A * (z * S - 1) / sqrt(C);
        chi = sqrt(y / C);
        dt_calc = (chi^3 * S + A * sqrt(y)) / sqrt(mu);
        if abs(dt_calc - dt)/dt < 1e-5, break; end
        if dt_calc < dt, z = z + 0.1; else, z = z - 0.1; end
        iter = iter + 1;
    end
    [C, S] = stumpff(z);
    y = r1_norm + r2_norm + A * (z * S - 1) / sqrt(C);
    f = 1 - y / r1_norm; g = A * sqrt(y / mu); g_dot = 1 - y / r2_norm;
    v1 = (r2 - f * r1) / g; v2 = (g_dot * r2 - r1) / g;
end

function [c, s] = stumpff(z)
    if z > 0, s = (sqrt(z) - sin(sqrt(z))) / z^1.5; c = (1 - cos(sqrt(z))) / z;
    elseif z < 0, s = (sinh(sqrt(-z)) - sqrt(-z)) / (-z)^1.5; c = (cosh(sqrt(-z)) - 1) / (-z);
    else, s = 1/6; c = 1/2; end
end