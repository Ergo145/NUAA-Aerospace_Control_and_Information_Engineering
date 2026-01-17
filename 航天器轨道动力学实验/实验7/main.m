% -------------------------------------------------------------------------
% 实验名称：近地小行星探测轨道设计
% 对应文档：第11章 近地小行星探测轨道设计 [cite: 1, 2]
% -------------------------------------------------------------------------
clc; clear; close all;

%% 1. 物理常数与参数设置
mu_sun = 1.32712440018e11;  % 太阳引力常数 (km^3/s^2)
mu_earth = 398600.4418;     % 地球引力常数 (km^3/s^2)
AU = 1.495978707e8;         % 天文单位 (km)

% 任务参数 (示例值，需根据实际星历调整)
R_parking = 6378.137 + 200; % 地球停泊轨道半径 (假设200km高度) [cite: 19]
R_reentry = 6378.137 + 110; % 再入高度半径 [cite: 24]

% 时间设置 (单位：天)
% 这里的数值仅为示例，对应文档中提到的参数化扫描 
launch_date = 0;       % 发射日期 (相对起点的天数)
transfer_out = 150;    % 去程飞行时间
stay_time = 30;        % 小行星停留时间
transfer_back = 150;   % 回程飞行时间

%% 2. 计算位置与速度矢量 (第1步 & 第3步) [cite: 13, 22]
% 注意：此处使用简化开普勒模型代替精密星历。
% 实际实验中应替换为 precision ephemeris (如 2013 WA44 的真实轨道根数)

t1 = launch_date;                      % 出发时刻
t2 = t1 + transfer_out;                % 到达小行星时刻
t3 = t2 + stay_time;                   % 离开小行星时刻
t4 = t3 + transfer_back;               % 返回地球时刻

% 获取天体状态 (r: 位置, v: 速度)
[r_e1, v_e1] = get_body_state('Earth', t1, mu_sun);      % 地球 @ t1
[r_a2, v_a2] = get_body_state('Asteroid', t2, mu_sun);   % 小行星 @ t2
[r_a3, v_a3] = get_body_state('Asteroid', t3, mu_sun);   % 小行星 @ t3
[r_e4, v_e4] = get_body_state('Earth', t4, mu_sun);      % 地球 @ t4

%% 3. 去程段：求解兰伯特问题 (第2步) [cite: 14]
dt_out = transfer_out * 86400; % 秒
[v_trans1_start, v_trans1_end] = lambert_solver(r_e1, r_a2, dt_out, mu_sun);

% 计算去程无穷远速度 v_infinity
v_inf_dep = norm(v_trans1_start - v_e1);

% 计算发射 Delta V (公式 11.1) [cite: 15]
% 停泊轨道速度
v_park = sqrt(mu_earth / R_parking);
% 要求的双曲线近地点速度
v_peri = sqrt(v_inf_dep^2 + 2*mu_earth/R_parking);
delta_v_dep = v_peri - v_park;

fprintf('--- 去程分析 ---\n');
fprintf('发射 C3 能量: %.2f km^2/s^2\n', v_inf_dep^2);
fprintf('发射 Delta V: %.4f km/s\n', delta_v_dep);

if delta_v_dep > 10
    warning('发射 Delta V 超过 10 km/s，任务不可行 [cite: 21]');
end

%% 4. 回程段：求解兰伯特问题 (第4步) [cite: 23]
dt_back = transfer_back * 86400; % 秒
[v_trans2_start, v_trans2_end] = lambert_solver(r_a3, r_e4, dt_back, mu_sun);

% 计算小行星处所需 Delta V (离开小行星)
% 假设需从与小行星同速变为转移轨道速度
delta_v_ast = norm(v_trans2_start - v_a3); 

% 计算回程无穷远速度 (相对地球)
v_inf_arr = norm(v_trans2_end - v_e4);

% 计算再入速度 (公式 11.2) [cite: 24]
v_reentry = sqrt(v_inf_arr^2 + 2*mu_earth/R_reentry);

%% 5. 总体评估 (第5步) [cite: 26]
total_delta_v = delta_v_dep + delta_v_ast;

fprintf('\n--- 回程分析 ---\n');
fprintf('小行星处 Delta V: %.4f km/s\n', delta_v_ast);
fprintf('再入速度: %.4f km/s\n', v_reentry);

fprintf('\n--- 总体结果 ---\n');
fprintf('总 Delta V: %.4f km/s\n', total_delta_v);

if total_delta_v < 10 && v_reentry < 14 % [cite: 47] 中提到的阈值
    fprintf('结论：轨道方案 **可行**。\n');
else
    fprintf('结论：轨道方案 **不可行** (超出约束)。\n');
end

%% 6. 绘图仿真 (对应图 11.1 - 中文图例版)
figure('Color', 'w'); hold on; axis equal; grid on;
view(3); xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('近地小行星探测往返轨道仿真');

% 绘制太阳
plot3(0,0,0, 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', '太阳','LineWidth',1);

% 绘制地球轨道和小行星轨道 (传入中文名称作为图例)
plot_orbit('Earth', mu_sun, 'b', '地球轨道');
plot_orbit('Asteroid', mu_sun, 'k', '小行星轨道');

% 绘制关键位置点
plot3(r_e1(1), r_e1(2), r_e1(3), 'bo', 'MarkerFaceColor','b', 'DisplayName', '地球出发','LineWidth',1);
plot3(r_a2(1), r_a2(2), r_a2(3), 'k^', 'MarkerFaceColor','k', 'DisplayName', '到达小行星','LineWidth',1);
plot3(r_e4(1), r_e4(2), r_e4(3), 'bs', 'MarkerFaceColor','c', 'DisplayName', '返回地球','LineWidth',1);

% 绘制转移轨迹
plot_transfer(r_e1, v_trans1_start, dt_out, mu_sun, 'r--', '去程转移');
plot_transfer(r_a3, v_trans2_start, dt_back, mu_sun, 'g--', '回程转移');

% 显示图例
legend('show', 'Location', 'northeast');

%% 7. 辅助函数
function plot_orbit(body_name, mu, color_spec, legend_name)
    % 绘制完整轨道周期
    % body_name: 用于计算 ('Earth' 或 'Asteroid')
    % legend_name: 图例显示的中文名称
    dt_step = 3600*24*5; % 5天一步
    period = 365.25 * 2 * 86400; % 粗略取2年
    t_span = 0:dt_step:period;
    points = zeros(length(t_span), 3);
    for k = 1:length(t_span)
        [r, ~] = get_body_state(body_name, t_span(k)/86400, mu);
        points(k, :) = r;
    end
    plot3(points(:,1), points(:,2), points(:,3), 'Color', color_spec, 'DisplayName', legend_name,'LineWidth',1);
end

function plot_transfer(r_start, v_start, dt, mu, style, name)
    % 积分并绘制转移轨道
    options = odeset('RelTol', 1e-6);
    [~, state] = ode45(@(t,y) two_body_ode(t,y,mu), [0, dt], [r_start, v_start], options);
    plot3(state(:,1), state(:,2), state(:,3), style, 'LineWidth', 1.5, 'DisplayName', name,'LineWidth',1);
end

function dydt = two_body_ode(~, y, mu)
    r = y(1:3);
    v = y(4:6);
    r_norm = norm(r);
    a = -mu * r / r_norm^3;
    dydt = [v; a];
end