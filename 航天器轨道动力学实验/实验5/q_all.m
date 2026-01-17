%% 实验三：二体问题意义下航天器轨道的数值解法
% 学院：航天学院
% 课程：航天器轨道力学
% 日期：2025年12月9日
% 参考文献：实验三.pdf [cite: 1-172]

clear; clc; close all;

%% --- 物理常数定义 ---
global mu
mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
Re = 6371;              % 地球平均半径 (km) [cite: 143]
omega_e = 7.292115e-5;  % 地球自转角速度 (rad/s)

%% ================================================================
%  任务 1: 利用欧拉法、改进欧拉法、RK4求解二体问题
%  [cite: 131]
% ================================================================
fprintf('正在执行任务 1: 比较数值积分方法...\n');

% 1. 初始状态 [cite: 134]
r0 = [-5102; -8228; -2105]; % km
v0 = [-4.348; 3.478; -2.846]; % km/s
y0 = [r0; v0]; % 状态向量 [x; y; z; vx; vy; vz]

% 2. 仿真设置
T_period = 2*pi*sqrt(norm(r0)^3/mu); % 估算轨道周期用于设定仿真时间
t_span = [0, 15000]; % 仿真 15000 秒 (约4小时)
h = 10; % 积分步长 10秒

% 3. 解算
[t_eu, y_eu] = solve_ode(@sys_model, t_span, y0, h, 'euler');
[t_imp, y_imp] = solve_ode(@sys_model, t_span, y0, h, 'improved_euler');
[t_rk4, y_rk4] = solve_ode(@sys_model, t_span, y0, h, 'rk4');

% 4. 绘图：三维轨迹对比
figure(1); set(gcf, 'Color', 'w', 'Name', 'Task 1: 3D Trajectory');
plot3(y_eu(:,1), y_eu(:,2), y_eu(:,3), 'g--', 'LineWidth', 1); hold on;
plot3(y_imp(:,1), y_imp(:,2), y_imp(:,3), 'b-.', 'LineWidth', 1.5);
plot3(y_rk4(:,1), y_rk4(:,2), y_rk4(:,3), 'r-', 'LineWidth', 1.5);
% 画地球
[X_e,Y_e,Z_e] = sphere(50); surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal; grid on; xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
legend('Euler', 'Improved Euler', 'RK4', 'Earth');
title('三种数值积分方法的三维轨道对比');
view(3);

% 5. 绘图：状态变量随时间变化 (以向径 r 的模长为例)
r_norm_eu = sqrt(sum(y_eu(:,1:3).^2, 2));
r_norm_rk4 = sqrt(sum(y_rk4(:,1:3).^2, 2));

figure(2); set(gcf, 'Color', 'w', 'Name', 'Task 1: State Variables');
plot(t_eu, r_norm_eu, 'g--'); hold on;
plot(t_rk4, r_norm_rk4, 'r-');
xlabel('Time (s)'); ylabel('Radius (km)');
legend('Euler', 'RK4');
title('轨道半径随时间变化 (误差累积示意)');
grid on;

%% ================================================================
%  任务 2: 分析积分步长对计算精度的影响
%  [cite: 132]
% ================================================================
fprintf('正在执行任务 2: 步长精度分析...\n');

step_sizes = [10, 60, 100, 300]; % 不同的积分步长
colors = {'r', 'b', 'g', 'k'};

figure(3); set(gcf, 'Color', 'w', 'Name', 'Task 2: Step Size Analysis');
subplot(1,2,1); hold on; title('不同步长下的三维轨迹 (Euler法)');
[X_e,Y_e,Z_e] = sphere(20); surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'c', 'EdgeColor', 'none', 'FaceAlpha', 0.3);
axis equal; grid on; view(3);

subplot(1,2,2); hold on; title('不同步长的能量守恒误差');
xlabel('Time (s)'); ylabel('Specific Energy (km^2/s^2)');

for i = 1:length(step_sizes)
    h_test = step_sizes(i);
    % 使用欧拉法最容易看出步长对精度的破坏（RK4太稳定了，大步长下效果依然很好）
    [t_test, y_test] = solve_ode(@sys_model, [0, 20000], y0, h_test, 'euler');
    
    % 绘制轨迹
    subplot(1,2,1);
    plot3(y_test(:,1), y_test(:,2), y_test(:,3), 'Color', colors{i}, 'DisplayName', sprintf('h=%ds', h_test));
    
    % 计算比机械能 E = v^2/2 - mu/r
    v_sq = sum(y_test(:,4:6).^2, 2);
    r_mag = sqrt(sum(y_test(:,1:3).^2, 2));
    Energy = v_sq/2 - mu./r_mag;
    
    % 绘制能量
    subplot(1,2,2);
    plot(t_test, Energy, 'Color', colors{i}, 'DisplayName', sprintf('h=%ds', h_test));
end
subplot(1,2,1); legend show;
subplot(1,2,2); legend show; grid on;

%% ================================================================
%  任务 3: 叠加等时间间隔点以分析速度变化 (开普勒第二定律)
%  [cite: 133]
% ================================================================
fprintf('正在执行任务 3: 速度变化分析...\n');

% 使用 RK4 的高精度结果
figure(4); set(gcf, 'Color', 'w', 'Name', 'Task 3: Velocity Visualization');
plot3(y_rk4(:,1), y_rk4(:,2), y_rk4(:,3), 'k-', 'LineWidth', 0.5); hold on;
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% 每隔固定时间（例如 500秒）画一个点
% 我们的步长 h=10s，所以间隔 50 个索引
interval_idx = 50; 
scatter3(y_rk4(1:interval_idx:end, 1), ...
         y_rk4(1:interval_idx:end, 2), ...
         y_rk4(1:interval_idx:end, 3), ...
         30, 'r', 'filled');

axis equal; grid on; view(3);
title('等时间间隔点分布图 (点稀疏处速度快-近地点，点密集处速度慢-远地点)');
xlabel('X'); ylabel('Y'); zlabel('Z');

%% ================================================================
%  任务 4: GPS 星座仿真
%  [cite: 140, 141, 142, 143]
% ================================================================
fprintf('正在执行任务 4: GPS 星座仿真...\n');

% GPS 参数
gps_h = 20200;          % 高度 (km) [cite: 143]
gps_a = Re + gps_h;     % 半长轴
gps_e = 0;              % 圆轨道 [cite: 143]
gps_i = deg2rad(55);    % 倾角 [cite: 143]
planes = 6;             % 轨道面数量
sats_per_plane = 4;     % 每个面的卫星数
gps_omega = 0;          % 近地点幅角 (圆轨道无所谓，设为0)

t_gps_span = [0, 43200]; % 仿真 12 小时 (约半天)
h_gps = 60;              % 步长 60秒

figure(5); set(gcf, 'Color', 'k', 'Name', 'Task 4: GPS Constellation 3D');
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0 0 1], 'EdgeColor', 'none'); hold on;
axis equal; grid on; box on; set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'ZColor', 'w');
view(3); title('GPS 星座 3D 构型', 'Color', 'w');

figure(6); set(gcf, 'Color', 'w', 'Name', 'Task 4: GPS Ground Track');
img = imread('https://upload.wikimedia.org/wikipedia/commons/8/83/Equirectangular_projection_SW.jpg'); % 在线加载地图底图，如无法加载请注释
image([-180 180], [90 -90], img); hold on; axis xy;
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
title('GPS 星座星下点轨迹 (Ground Track)'); grid on;
xlim([-180 180]); ylim([-90 90]);

colors_plane = hsv(planes);

for k = 1:planes
    RAAN = deg2rad((k-1) * 60); % 各轨道面升交点赤经相差 60度 [cite: 143]
    
    for j = 1:sats_per_plane
        % 简单的相位分布：平近点角 M 均匀分布
        M0 = deg2rad((j-1) * (360/sats_per_plane)); 
        
        % 轨道六根数转位置速度 (COE -> RV)
        [r_gps0, v_gps0] = coe2rv(mu, gps_a, gps_e, gps_i, RAAN, gps_omega, M0);
        y_gps0 = [r_gps0; v_gps0];
        
        % 数值积分 (RK4)
        [t_gps, y_out] = solve_ode(@sys_model, t_gps_span, y_gps0, h_gps, 'rk4');
        
        % 绘制 3D 轨道 (只画第一颗卫星代表该轨道面，避免线条过多)
        if j == 1
            figure(5);
            plot3(y_out(:,1), y_out(:,2), y_out(:,3), 'Color', colors_plane(k,:), 'LineWidth', 0.5);
        end
        % 绘制当前卫星位置点
        figure(5);
        plot3(y_out(end,1), y_out(end,2), y_out(end,3), '.', 'MarkerSize', 15, 'Color', colors_plane(k,:));
        
        % --- 计算星下点 (Ground Track) ---
        % 需要将 ECI 坐标转换为 ECEF 坐标
        % formula: lambda = atan2(y,x) - omega_e * t
        
        % 预分配
        lon = zeros(length(t_gps), 1);
        lat = zeros(length(t_gps), 1);
        
        for ti = 1:length(t_gps)
            r_eci = y_out(ti, 1:3)';
            t_curr = t_gps(ti);
            
            % ECI 转 ECEF 旋转矩阵 [cite: 80] (绕Z轴旋转)
            theta_g = omega_e * t_curr;
            R_z = [cos(theta_g), sin(theta_g), 0;
                  -sin(theta_g), cos(theta_g), 0;
                   0,            0,            1];
            r_ecef = R_z * r_eci;
            
            % 计算经纬度 [cite: 90, 91]
            r_mag = norm(r_ecef);
            phi = asin(r_ecef(3) / r_mag); % 纬度
            lambda = atan2(r_ecef(2), r_ecef(1)); % 经度
            
            lon(ti) = rad2deg(lambda);
            lat(ti) = rad2deg(phi);
        end
        
        % 绘制星下点
        figure(6);
        plot(lon, lat, '.', 'MarkerSize', 2, 'Color', colors_plane(k,:));
    end
end

fprintf('所有实验任务完成。\n');


%% ================================================================
%  辅助函数区域
% ================================================================

% 1. 二体动力学模型 
function dydt = sys_model(~, y)
    global mu
    r_vec = y(1:3);
    v_vec = y(4:6);
    r = norm(r_vec);
    
    a_vec = -mu * r_vec / r^3;
    
    dydt = [v_vec; a_vec];
end

% 2. 通用 ODE 求解器封装
function [t, y] = solve_ode(fun, t_span, y0, h, method)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    
    for i = 1:length(t)-1
        switch method
            case 'euler' % 
                y(i+1, :) = step_euler(fun, t(i), y(i, :)', h);
            case 'improved_euler' % 
                y(i+1, :) = step_improved_euler(fun, t(i), y(i, :)', h);
            case 'rk4' % 
                y(i+1, :) = step_rk4(fun, t(i), y(i, :)', h);
        end
    end
end

% 3. 欧拉法步进 
function y_next = step_euler(fun, t, y, h)
    k1 = fun(t, y);
    y_next = y + h * k1;
end

% 4. 改进欧拉法步进 
function y_next = step_improved_euler(fun, t, y, h)
    k1 = fun(t, y);
    y_predict = y + h * k1;
    k2 = fun(t + h, y_predict);
    y_next = y + (h/2) * (k1 + k2);
end

% 5. 四阶龙格库塔法步进 
function y_next = step_rk4(fun, t, y, h)
    k1 = fun(t, y);
    k2 = fun(t + 0.5*h, y + 0.5*h*k1);
    k3 = fun(t + 0.5*h, y + 0.5*h*k2);
    k4 = fun(t + h, y + h*k3);
    y_next = y + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

% 6. 轨道六根数转状态向量 (COE -> RV)
function [r_vec, v_vec] = coe2rv(mu, a, e, i, Omega, omega, M)
    % 计算偏近点角 E (对于 e=0, E=M)
    if e < 1e-6
        E = M;
    else
        % 简单牛顿迭代解开普勒方程 M = E - e*sin(E)
        E = M; 
        for iter = 1:10
            E = M + e*sin(E);
        end
    end
    
    % 在轨道平面坐标系 (Perifocal Frame) 中的位置和速度
    p = a * (1 - e^2);
    r_pqw = [a*(cos(E)-e); a*sqrt(1-e^2)*sin(E); 0];
    v_pqw = sqrt(mu/p) * [-sin(E); sqrt(1-e^2)*cos(E); 0];
    
    % 旋转矩阵 (3-1-3 旋转: -Omega, -i, -omega 的逆变换)
    R1 = [cos(Omega) -sin(Omega) 0; sin(Omega) cos(Omega) 0; 0 0 1];
    R2 = [1 0 0; 0 cos(i) -sin(i); 0 sin(i) cos(i)];
    R3 = [cos(omega) -sin(omega) 0; sin(omega) cos(omega) 0; 0 0 1];
    
    Q = R1 * R2 * R3;
    
    r_vec = Q * r_pqw;
    v_vec = Q * v_pqw;
end