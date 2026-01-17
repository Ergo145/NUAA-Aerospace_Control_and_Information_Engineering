clc; clear; close all;

%% --- 1. 参数定义 ---
global mu
mu = 398600.4418;       
Re = 6371;              
omega_e = 7.292115e-5;  

% GPS 参数
gps_a = Re + 20200;     
gps_e = 0;              
gps_i = deg2rad(55);    
planes = 6;             
sats_per_plane = 4;     

% 仿真设置
h_step = 60;             % 积分步长 60s
t_total = 86400;         % 仿真总时长 24 小时
t_span = 0:h_step:t_total; 
len_t = length(t_span);  

%% 2. 核心计算

total_sats = planes * sats_per_plane;
% 结构体存储 24h 数据
const_data = struct('r_eci', {}, 'lla', {}, 'color', {}, 'lla_plot', {});
sat_idx = 0;
colors_plane = hsv(planes); 

for k = 1:planes
    RAAN = deg2rad((k-1) * 60); 
    for j = 1:sats_per_plane
        sat_idx = sat_idx + 1;
        M0 = deg2rad((j-1) * (360/sats_per_plane));
        
        % 1. 初始状态 & 积分
        [r0, v0] = coe2rv(mu, gps_a, gps_e, gps_i, RAAN, 0, M0);
        [~, y_out] = solve_ode_rk4(@sys_model, t_span, [r0; v0], h_step);
        
        r_eci_full = y_out(:, 1:3);
        const_data(sat_idx).r_eci = r_eci_full;
        const_data(sat_idx).color = colors_plane(k, :);
        
        % 2. 计算 24h LLA
        r_eci = r_eci_full;
        theta_g = omega_e * t_span(:);
        c_g = cos(theta_g); s_g = sin(theta_g);
        x_ecef = c_g .* r_eci(:,1) + s_g .* r_eci(:,2);
        y_ecef = -s_g .* r_eci(:,1) + c_g .* r_eci(:,2);
        z_ecef = r_eci(:,3);
        r_mag = sqrt(x_ecef.^2 + y_ecef.^2 + z_ecef.^2);
        lat = rad2deg(asin(z_ecef ./ r_mag));
        lon = rad2deg(atan2(y_ecef, x_ecef));
        const_data(sat_idx).lla = [lon, lat];
        
        % 3. 处理 24h 轨迹的 2D 跨界断点
        lon_plot = lon;
        lat_plot = lat;
        diff_lon = diff(lon_plot);
        jump_indices = find(abs(diff_lon) > 300); 
        lon_plot(jump_indices) = NaN;
        lat_plot(jump_indices) = NaN;
        
        const_data(sat_idx).lla_plot = [lon_plot, lat_plot];
    end
end

%% 3. 场景初始化

% === Figure 1: 3D 视图 ===
f3d = figure(1); 
set(f3d, 'Color', 'w', 'Name', 'Task 4: GPS 24h (30s World Time)', 'Position', [50, 100, 800, 600]);
clf; axis equal; grid on; box on; hold on;
view(3); title('GPS 24h 动态仿真 (30s 播放)', 'FontSize', 14);
xlim([-35000 35000]); ylim([-35000 35000]); zlim([-35000 35000]);
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', [0.9 0.9 1], 'EdgeColor', [0.8 0.8 0.8]); 

% === Figure 2: 2D 地图视图 ===
f2d = figure(2);
set(f2d, 'Color', 'w', 'Name', 'Task 4: GPS Ground Track (30s World Time)', 'Position', [900, 100, 800, 600]);
clf; hold on; axis xy; grid on;
xlabel('Longitude (deg)'); ylabel('Latitude (deg)');
xlim([-180 180]); ylim([-90 90]); title('GPS 24h 星下点轨迹 (30s 播放)');

try
    img = imread('Equirectangular_projection_SW.jpg'); 
    image([-180 180], [90 -90], img);
    alpha(0.6); 
catch
    text(0,0,'Map Image Not Found', 'HorizontalAlignment','center');
end

h_sats_3d = gobjects(total_sats, 1);
h_sats_2d = gobjects(total_sats, 1);

for i = 1:total_sats
    c = const_data(i).color;
    
    figure(1);
    plot3(const_data(i).r_eci(:,1), const_data(i).r_eci(:,2), const_data(i).r_eci(:,3), ...
        'Color', [c, 0.4], 'LineWidth', 1);
    figure(2);
    plot(const_data(i).lla_plot(:,1), const_data(i).lla_plot(:,2), ...
        'Color', [c, 0.5], 'LineWidth', 1);
    
    % 初始化动态点
    figure(1);
    h_sats_3d(i) = plot3(0, 0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    figure(2);
    h_sats_2d(i) = plot(0, 0, 's', 'MarkerSize', 6, 'MarkerFaceColor', c, 'MarkerEdgeColor', 'k');
end

%% 4. 动画循环

t_idx = 1;
speed_factor = 2;

while ishandle(f3d) && ishandle(f2d)
    
    % 批量更新所有卫星位置
    for i = 1:total_sats
        % 3D 更新
        p3 = const_data(i).r_eci(t_idx, :);
        set(h_sats_3d(i), 'XData', p3(1), 'YData', p3(2), 'ZData', p3(3));
        
        % 2D 更新
        p2 = const_data(i).lla(t_idx, :);
        set(h_sats_2d(i), 'XData', p2(1), 'YData', p2(2));
    end
    
    % 更新标题时间
    h_curr = t_span(t_idx)/3600;
    title(f3d.CurrentAxes, sprintf('GPS 3D View | Sim Time: %.1f / 24.0 h', h_curr));
    title(f2d.CurrentAxes, sprintf('GPS Ground Track | Sim Time: %.1f / 24.0 h', h_curr));
    
    drawnow limitrate; 
    
    % 循环逻辑
    t_idx = t_idx + speed_factor;
    if t_idx > len_t
        t_idx = 1; % 回到起点循环
    end
end

%% 辅助函数
function [t, y] = solve_ode_rk4(fun, t_span, y0, h)
    t = t_span(:);
    N = length(t);
    y = zeros(N, length(y0));
    y(1, :) = y0';
    y_curr = y0;
    for i = 1:N-1
        k1 = fun(t(i), y_curr);
        k2 = fun(t(i) + 0.5*h, y_curr + 0.5*h*k1);
        k3 = fun(t(i) + 0.5*h, y_curr + 0.5*h*k2);
        k4 = fun(t(i) + h, y_curr + h*k3);
        y_curr = y_curr + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
        y(i+1, :) = y_curr';
    end
end

function dydt = sys_model(~, y)
    global mu
    r = norm(y(1:3));
    dydt = [y(4:6); -mu * y(1:3) / r^3];
end

function [r, v] = coe2rv(mu, a, e, i, Om, om, M)
    E = M; 
    r_pqw = [a*cos(E); a*sin(E); 0];
    v_pqw = sqrt(mu/a) * [-sin(E); cos(E); 0];
    R = [cos(Om) -sin(Om) 0; sin(Om) cos(Om) 0; 0 0 1] * ...
        [1 0 0; 0 cos(i) -sin(i); 0 sin(i) cos(i)] * ...
        [cos(om) -sin(om) 0; sin(om) cos(om) 0; 0 0 1];
    r = R * r_pqw; v = R * v_pqw;
end