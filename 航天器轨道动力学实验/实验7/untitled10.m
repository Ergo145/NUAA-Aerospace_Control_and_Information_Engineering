% -------------------------------------------------------------------------
% 实验名称：近地小行星探测轨道设计 (Itokawa Mission)
% 核心算法：Code B (解析法，速度极快)
% 绘图风格：Code A (特定配色、标记、图例)
% 输出格式：严格匹配要求
% -------------------------------------------------------------------------
clc; clear; close all;

%% 1. 基础物理参数 (完全来自 Code B)
mu_sun = 1.32712440018e11;      % 太阳引力常数 (m^3/s^2)
mu_earth = 3.986004418e14;      % 地球引力常数 (m^3/s^2)
AU = 149597870700;              % 1天文单位 (m)
R_parking = 6500e3;             % 地球停泊轨道半径 (m)
R_earth_atm = 6717.5e3 + 110e3; % 再入大气层参考半径 (m)

% 约束条件
max_total_DV = 10;              % 最大允许总ΔV (km/s)
max_reentry_V = 14;             % 最大允许再入速度 (km/s)

% 任务时间参数
t_launch = datetime(2027, 1, 1);
t_arrive_ast = t_launch + days(180);
t_depart_ast = t_arrive_ast + days(30);
t_return_earth = t_depart_ast + days(200);

% 儒略日转换
JD_launch = juliandate(t_launch);
JD_arrive = juliandate(t_arrive_ast);
JD_depart = juliandate(t_depart_ast);
JD_return = juliandate(t_return_earth);
JD0 = 2451545.0; 

%% 2. 轨道根数定义 (Code B)
% Itokawa
a_ast = 1.324168468888213 * AU;    
e_ast = 0.280487226585832;          
i_ast = deg2rad(1.628635888032334); 
Omega_ast = deg2rad(69.2960029230903); 
omega_ast = deg2rad(208.999523603423); 
M0_ast = deg2rad(358.653144450033);    

% Earth
a_earth = 1.000000112461118 * AU;   
e_earth = 0.016708617433182;        
i_earth = deg2rad(0.000050050888346); 
Omega_earth = deg2rad(-11.260640891); 
omega_earth = deg2rad(102.947190494); 

%% 3. 计算状态矢量 (Code B 逻辑)
[R_earth_launch, V_earth_launch] = kepler2cart(a_earth, e_earth, i_earth, Omega_earth, omega_earth, JD_launch, JD0, mu_sun, M0_ast);
[R_earth_return, V_earth_return] = kepler2cart(a_earth, e_earth, i_earth, Omega_earth, omega_earth, JD_return, JD0, mu_sun, M0_ast);

[R_ast_arrive, V_ast_arrive] = kepler2cart(a_ast, e_ast, i_ast, Omega_ast, omega_ast, JD_arrive, JD0, mu_sun, M0_ast);
[R_ast_depart, V_ast_depart] = kepler2cart(a_ast, e_ast, i_ast, Omega_ast, omega_ast, JD_depart, JD0, mu_sun, M0_ast);

%% 4. 求解兰伯特问题 & 获取转移轨迹点 (Code B 逻辑 - 极速)
% 6.1 去程
tof_depart = (JD_arrive - JD_launch) * 86400; 
[V_depart_earth, V_arrive_ast, orbit_depart] = lambert(R_earth_launch, R_ast_arrive, tof_depart, mu_sun);

v_a_inf = norm(V_depart_earth - V_earth_launch);
DV_depart = sqrt(v_a_inf^2 + 2*mu_earth/R_parking) - sqrt(mu_earth/R_parking);

% 6.2 回程
tof_return = (JD_return - JD_depart) * 86400; 
[V_depart_ast, V_arrive_earth, orbit_return] = lambert(R_ast_depart, R_earth_return, tof_return, mu_sun);

DV_return = norm(V_depart_ast - V_ast_depart);
total_DV = DV_depart + DV_return;
v_e_inf = norm(V_arrive_earth - V_earth_return);
V_reentry = sqrt(v_e_inf^2 + 2*mu_earth/R_earth_atm);

%% 5. 提前生成整轨点集 (Code B 逻辑)
n_points = 200;
[points_earth_orbit] = get_orbit_points(a_earth, e_earth, i_earth, Omega_earth, omega_earth, mu_sun, n_points);
[points_ast_orbit] = get_orbit_points(a_ast, e_ast, i_ast, Omega_ast, omega_ast, mu_sun, n_points);

%% 6. 输出结果 (严格匹配要求格式)
fprintf('========== 轨道计算结果 ==========\n');
fprintf('发射时间：%s\n', datestr(t_launch));
fprintf('到达小行星时间：%s\n', datestr(t_arrive_ast));
fprintf('离开小行星时间：%s\n', datestr(t_depart_ast));
fprintf('返回地球时间：%s\n', datestr(t_return_earth));
fprintf('去程ΔV：%.3f km/s\n', DV_depart/1000);
fprintf('回程ΔV：%.3f km/s\n', DV_return/1000);
fprintf('总ΔV消耗：%.3f km/s\n', total_DV/1000);
fprintf('再入速度：%.3f km/s\n', V_reentry/1000);

% 可行性判断逻辑
if (total_DV/1000 < max_total_DV) && (V_reentry/1000 < max_reentry_V)
    fprintf('结论：轨道可行！\n');
else
    fprintf('结论：轨道不可行（超出ΔV或再入速度限制）\n');
end
fprintf('===================================\n');

%% 7. 绘图仿真 (Code A 视觉风格)
scale = 1/1000; % 米转千米

figure('Color', 'w'); hold on; axis equal; grid on;
view(3); xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
title('近地小行星探测往返轨道仿真');

% 7.1 绘制太阳
plot3(0,0,0, 'yo', 'MarkerSize', 10, 'MarkerFaceColor', 'y', 'DisplayName', '太阳', 'LineWidth', 1);

% 7.2 绘制轨道
plot3(points_earth_orbit(:,1)*scale, points_earth_orbit(:,2)*scale, points_earth_orbit(:,3)*scale, ...
    'b', 'DisplayName', '地球轨道', 'LineWidth', 1);
plot3(points_ast_orbit(:,1)*scale, points_ast_orbit(:,2)*scale, points_ast_orbit(:,3)*scale, ...
    'k', 'DisplayName', '小行星轨道', 'LineWidth', 1);

% 7.3 绘制关键点
p_e1 = R_earth_launch * scale;
p_a2 = R_ast_arrive * scale;
p_a3 = R_ast_depart * scale;
p_e4 = R_earth_return * scale;

plot3(p_e1(1), p_e1(2), p_e1(3), 'bo', 'MarkerFaceColor','b', 'DisplayName', '地球出发', 'LineWidth', 1);
plot3(p_a2(1), p_a2(2), p_a2(3), 'k^', 'MarkerFaceColor','k', 'DisplayName', '到达小行星', 'LineWidth', 1);
plot3(p_a3(1), p_a3(2), p_a3(3), 'kv', 'MarkerFaceColor','k', 'DisplayName', '离开小行星', 'LineWidth', 1);
plot3(p_e4(1), p_e4(2), p_e4(3), 'bs', 'MarkerFaceColor','c', 'DisplayName', '返回地球', 'LineWidth', 1);

% 7.4 绘制转移轨迹 (Code B生成点，Code A样式)
plot3(orbit_depart.r(:,1)*scale, orbit_depart.r(:,2)*scale, orbit_depart.r(:,3)*scale, ...
    'r--', 'LineWidth', 1.5, 'DisplayName', '去程转移');
plot3(orbit_return.r(:,1)*scale, orbit_return.r(:,2)*scale, orbit_return.r(:,3)*scale, ...
    'g--', 'LineWidth', 1.5, 'DisplayName', '回程转移');

legend('show', 'Location', 'northeast');

%% 8. 辅助函数 (Code B 原版 - 极速核心)

function [r, v] = kepler2cart(a, e, i, Omega, omega, JD, JD0, mu, M0)
    % 计算平近点角M
    n = sqrt(mu / a^3); 
    dt = (JD - JD0) * 86400; 
    M = M0 + n * dt; 
    M = mod(M, 2*pi); 
    
    % 求解开普勒方程
    E = M; 
    for iter = 1:50
        f_E = E - e*sin(E) - M;
        f_E_deriv = 1 - e*cos(E);
        E = E - f_E / f_E_deriv;
        if abs(f_E) < 1e-10, break; end
    end
    
    % 真近点角f
    f = 2*atan2(sqrt(1+e)*sin(E/2), sqrt(1-e)*cos(E/2));
    
    % 轨道平面内位置/速度
    r_orbit = a*(1 - e^2)/(1 + e*cos(f)); 
    x_orbit = r_orbit * cos(f);
    y_orbit = r_orbit * sin(f);
    
    h = sqrt(mu * a * (1 - e^2)); 
    vx_orbit = mu/h*(-sin(f));    
    vy_orbit = mu/h*(e + cos(f));
    
    % 旋转矩阵
    R_omega = [cos(omega), -sin(omega), 0; sin(omega), cos(omega), 0; 0, 0, 1];
    R_i = [1, 0, 0; 0, cos(i), -sin(i); 0, sin(i), cos(i)];
    R_Omega = [cos(Omega), -sin(Omega), 0; sin(Omega), cos(Omega), 0; 0, 0, 1];
    R = R_Omega * R_i * R_omega;
    
    r = (R * [x_orbit; y_orbit; 0])';
    v = (R * [vx_orbit; vy_orbit; 0])';
end

function [v1, v2, orbit] = lambert(r1, r2, tof, mu)
    % Code B 的兰伯特求解器，包含解析法生成轨迹点
    r1_mag = norm(r1);
    r2_mag = norm(r2);
    delta_r = r2 - r1;
    delta_r_mag = norm(delta_r);
    
    cos_gamma = dot(r1, r2)/(r1_mag*r2_mag);
    cos_gamma = max(min(cos_gamma, 1), -1);
    gamma = acos(cos_gamma);
    temp = cross(r1, r2);
    if temp(3) < 0, gamma = 2*pi - gamma; end
    
    x = 0.1; 
    for iter = 1:50
        y = r1_mag + r2_mag + (delta_r_mag^2)/(2*sqrt(r1_mag*r2_mag)) * x^2 * (1 - x*sqrt(mu)/sqrt((r1_mag + r2_mag)^3)*tof);
        y = max(y, 1e-3); 
        
        if x > 0
            s = sqrt(x); S = (s - sin(s))/s^3; C = (1 - cos(s))/s^2;
        elseif x < 0
            s = sqrt(-x); S = (sinh(s) - s)/s^3; C = (cosh(s) - 1)/(-x);
        else
            S = 1/6; C = 1/2;
        end
        
        t = (sqrt(y^3)/sqrt(mu)) * (x^2*C - (1 - r1_mag/sqrt(y))*x*sqrt(C*(2 - x^2*C)));
        dt = t - tof;
        if abs(dt) < 1e-8, break; end
        dt_dx = (sqrt(y^3)/sqrt(mu)) * (2*x*C + x^2*(1 - x^2*C)/(2*x) - (1 - r1_mag/sqrt(y))*(sqrt(C*(2 - x^2*C)) + x*(1 - x^2*C)/(2*x)*sqrt((2 - x^2*C)/C)/2));
        x = x - dt/dt_dx;
    end
    
    f = 1 - y/r1_mag;
    g = tof - sqrt(y^3/mu)*x^2*S;
    g_dot = 1 - y/r2_mag;
    v1 = (r2 - f*r1)/g;
    v2 = (g_dot*r2 - r1)/g;
    
    % 解析法生成轨迹点 (极速核心)
    n_points = 50;
    tau = linspace(0, 1, n_points);
    orbit.r = zeros(n_points, 3);
    for i = 1:n_points
        x_tau = x * tau(i);
        if x_tau > 0
            s_tau = sqrt(x_tau); S_tau = (s_tau - sin(s_tau))/s_tau^3; C_tau = (1 - cos(s_tau))/s_tau^2;
        elseif x_tau < 0
            s_tau = sqrt(-x_tau); S_tau = (sinh(s_tau) - s_tau)/s_tau^3; C_tau = (cosh(s_tau) - 1)/(-x_tau);
        else
            S_tau = 1/6; C_tau = 1/2;
        end
        y_tau = r1_mag + r2_mag + (delta_r_mag^2)/(2*sqrt(r1_mag*r2_mag)) * x_tau^2 * (1 - x_tau*sqrt(mu)/sqrt((r1_mag + r2_mag)^3)*tof);
        f_tau = 1 - y_tau/r1_mag;
        g_tau = tof*tau(i) - sqrt(y_tau^3/mu)*x_tau^2*S_tau;
        orbit.r(i,:) = (f_tau*r1 + g_tau*v1)';
    end
end

function [R_orbit] = get_orbit_points(a, e, i, Omega, omega, mu, n_points)
    % Code B 的整轨生成函数
    R_orbit = zeros(n_points, 3);
    M_list = linspace(0, 2*pi, n_points); 
    for j = 1:n_points
        M = M_list(j);
        E = M;
        for iter = 1:20
            f_E = E - e*sin(E) - M;
            E = E - f_E / (1 - e*cos(E));
            if abs(f_E) < 1e-10, break; end
        end
        f = 2*atan2(sqrt(1+e)*sin(E/2), sqrt(1-e)*cos(E/2));
        r_orbit = a*(1 - e^2)/(1 + e*cos(f));
        x_orbit = r_orbit * cos(f);
        y_orbit = r_orbit * sin(f);
        
        R_omega = [cos(omega), -sin(omega), 0; sin(omega), cos(omega), 0; 0, 0, 1];
        R_i = [1, 0, 0; 0, cos(i), -sin(i); 0, sin(i), cos(i)];
        R_Omega = [cos(Omega), -sin(Omega), 0; sin(Omega), cos(Omega), 0; 0, 0, 1];
        R = R_Omega * R_i * R_omega;
        R_orbit(j,:) = (R * [x_orbit; y_orbit; 0])';
    end
end