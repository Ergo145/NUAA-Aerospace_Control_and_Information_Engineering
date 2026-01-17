clear; clc; close all;

% --- 1. 常量与基准参数设置 ---
mu = 398600.4418; % 地球引力常数 km^3/s^2
we = 7.292115e-5; % 地球自转角速度 rad/s
Re = 6378;        % 地球平均半径 km

% 经典轨道六根数 (a, e, i, Omega, omega, nu) - 用于定义轨道
coe_classic = [7000, 0.05, 45, 0, 0, 0]; 

a = coe_classic(1); 
e = coe_classic(2); 
i_deg = coe_classic(3);
M0 = 0; % 初始平近点角为0

% 计算轨道周期 T
T = 2 * pi * sqrt(a^3 / mu); % 秒
fprintf('轨道参数：a=%.1f km, e=%.2f, i=%.1f°\n', a, e, i_deg);
fprintf('轨道周期 T = %.2f 分钟\n', T/60);

% 定义要绘制的周期数
num_periods_to_plot = [1, 2, 3]; 

% --- 2. 循环绘图 ---
for num_periods = num_periods_to_plot
    T_total = num_periods * T;
    
    % 调用修正后的计算函数
    [lon, lat] = calculate_ground_track_h(T_total, coe_classic, mu, we, M0);
    
    % 绘制星下点轨迹
    figure('Name', ['Ground Track - ' num2str(num_periods) ' Periods'], 'Color', 'w');
    
    % 绘制海岸线
    try
        load coastlines; 
        plot(coastlon, coastlat, 'k', 'Color', [0.7 0.7 0.7]); hold on;
    catch
        % 如果没有 coastlines 数据，留空
    end

    plot(lon, lat, 'b-', 'LineWidth', 1.5, 'DisplayName', '轨迹'); hold on;
    
    % 标记起点和终点
    plot(lon(1), lat(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
    plot(lon(end), lat(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '终点');
    
    axis([-180 180 -90 90]);
    xticks(-180:60:180);
    yticks(-90:30:90);
    grid on;
    xlabel('经度 (deg)'); 
    ylabel('纬度 (deg)');
    title(['星下点轨迹 (仿真 ' num2str(num_periods) ' 个周期, T=' num2str(T/60, '%.2f') ' min)']);
    legend('Location', 'southwest');
    
    % 计算赤道跨度
    delta_lambda = rad2deg(we * T);
    fprintf('%d 个周期绘图完成。赤道经度跨度 (T) 约为 %.2f 度。\n', num_periods, delta_lambda);
end


% --- 3. 核心计算函数 (使用 h 作为输入) ---
function [lon, lat] = calculate_ground_track_h(T_total, coe_classic, mu, we, M0)
    % 提取经典参数
    a = coe_classic(1); 
    e = coe_classic(2);
    i = deg2rad(coe_classic(3)); % 倾角转弧度
    Omega = deg2rad(coe_classic(4)); % 升交点赤经转弧度
    omega = deg2rad(coe_classic(5)); % 近地点幅角转弧度

    % 关键修改：计算 h
    p = a * (1 - e^2);
    h = sqrt(mu * p); % 轨道角动量
    
    t_span = 0 : 30 : T_total; % 每30秒一个点
    lat_calc = []; lon_calc = [];
    n = sqrt(mu/a^3); % 平均角速度
    
    for t = t_span
        
        % 1. 求解开普勒方程求真近点角 TA (或 nu)
        M = M0 + n * t;   % 当前平近点角
        E = M; 
        for iter = 1:5
            E = M + e * sin(E);
        end
        
        sin_TA = (sqrt(1-e^2)*sin(E)) / (1-e*cos(E));
        cos_TA = (cos(E)-e) / (1-e*cos(E));
        TA = atan2(sin_TA, cos_TA); % 真近点角 (弧度)
        
        % 2. 构造新格式的 coe_h (h, e, RA, incl, w, TA)
        coe_h = [h, e, Omega, i, omega, TA];
        
        % 3. 调用 sv_from_coe
        [r_eci, ~] = sv_from_coe(coe_h); % r_eci, v_eci 是行向量
        r_eci = r_eci'; % 转为列向量便于矩阵运算
        
        % 4. 转换到 ECEF (地固系)
        theta_g = we * t; % 地球自转角度
        
        R_z = [cos(theta_g) sin(theta_g) 0;
              -sin(theta_g) cos(theta_g) 0;
               0            0           1];
               
        r_ecef = R_z * r_eci;
        
        % 5. 计算经纬度
        r_mag = norm(r_ecef);
        phi = asin(r_ecef(3) / r_mag);     % 纬度 (弧度)
        lambda = atan2(r_ecef(2), r_ecef(1)); % 经度 (弧度)
        
        lat_calc = [lat_calc, rad2deg(phi)];
        lon_calc = [lon_calc, rad2deg(lambda)];
    end
    
    % 6. 处理经度跳变
    for k = 2:length(lon_calc)
        if abs(lon_calc(k) - lon_calc(k-1)) > 100
            lon_calc(k-1) = NaN; 
        end
    end
    lon = lon_calc;
    lat = lat_calc;
end