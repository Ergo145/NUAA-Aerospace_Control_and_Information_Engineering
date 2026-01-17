clc;clear;close all;

%% --- 物理常数定义 ---
global mu
mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
Re = 6371;              % 地球平均半径 (km) 
omega_e = 7.292115e-5;  % 地球自转角速度 (rad/s)

% GPS 参数
gps_h = 20200;          % 高度 (km)
gps_a = Re + gps_h;     % 半长轴
gps_e = 0;              % 圆轨道
gps_i = deg2rad(55);    % 倾角
planes = 6;             % 轨道面数量
sats_per_plane = 4;     % 每个面的卫星数
gps_omega = 0;          % 近地点幅角 (圆轨道无所谓，设为0)

t_gps_span = [0, 43200]; % 仿真 12 小时 (约半天)
h_gps = 60;              % 步长 60秒

figure(5); set(gcf, 'Color', 'w', 'Name', 'Task 4: GPS Constellation 3D');
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0 0 1], 'EdgeColor', 'none'); hold on;
axis equal; grid on; box on; set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
view(3); title('GPS 星座 3D 构型', 'Color', 'k')

figure(6); set(gcf, 'Color', 'w', 'Name', 'Task 4: GPS Ground Track');
img = imread('Equirectangular_projection_SW.jpg'); 
image([-180 180], [90 -90], img); hold on; axis xy;
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
title('GPS 星座星下点轨迹 (Ground Track)'); grid on;
xlim([-180 180]); ylim([-90 90]);

colors_plane = hsv(planes);

for k = 1:planes
    RAAN = deg2rad((k-1) * 60); % 各轨道面升交点赤经相差 60度

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
        plot(lon, lat, '.', 'MarkerSize', 2, 'Color', colors_plane(k,:),"LineWidth",2);
    end
end

% 1. 二体动力学模型
function dydt = sys_model(~, y)
global mu
r_vec = y(1:3);
v_vec = y(4:6);
r = norm(r_vec);

a_vec = -mu * r_vec / r^3;

dydt = [v_vec; a_vec];
end

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