function Orbit_RK_Compare_Enhanced
    clc; clear; close all;
    
    % 全局变量定义
    global mu
    mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
    
    % --------------------------
    % 1. 初始参数设置
    % --------------------------
    fprintf('===== 两种积分方法对比 (RK4 vs RKF78) =====\n');
    
    % 初始状态
    R0 = [-5102, -8228, -2105];   % 初始位置 (km)
    V0 = [-4.348, 3.478, -2.846]; % 初始速度 (km/s)
    
    % 求解轨道根数并计算周期
    orbit_coe = coe_caculate(R0, V0);
    a = orbit_coe(7);             
    T_period = 2 * pi * sqrt(a^3 / mu); 
    
    % 积分设置
    h = 0.5;                      % 时间步长 (s)
    t0 = 0;                       
    tf = T_period;                % 仿真一个周期
    tout = t0:h:tf;               
    N = length(tout);             
    
    fprintf('仿真周期: %.2f 秒\n', tf);

    % --------------------------
    % 2. RK4 (四阶龙格-库塔) 求解
    % --------------------------
    Xout_rk4 = zeros(N, 6);
    R = R0; V = V0;
    Xout_rk4(1, :) = [R, V];
    
    for k = 2:N
        [k1r, k1v] = stateequation(R, V);
        [k2r, k2v] = stateequation(R + 0.5*h*k1r, V + 0.5*h*k1v);
        [k3r, k3v] = stateequation(R + 0.5*h*k2r, V + 0.5*h*k2v);
        [k4r, k4v] = stateequation(R + h*k3r, V + h*k3v);
        
        R = R + h*(k1r + 2*k2r + 2*k3r + k4r)/6;
        V = V + h*(k1v + 2*k2v + 2*k3v + k4v)/6;
        
        Xout_rk4(k, :) = [R, V];
    end
    
    % --------------------------
    % 3. RKF7(8) 求解
    % --------------------------
    Xout_rkf78 = zeros(N, 6);  
    Xout_rkf78(1, :) = [R0, V0]; 
    
    % 简化的系数矩阵构建 (为节省篇幅，功能与之前相同)
    beta = zeros(13,13);
    beta(2,1) = 2/27;
    beta(3,1:2) = [1/36, 1/12];
    beta(4,1:3) = [1/24, 0, 1/8];
    beta(5,1:4) = [5/12, 0, -25/16, 25/16];
    beta(6,1:5) = [1/20, 0, 0, 1/4, 1/5];
    beta(7,1:6) = [-25/108, 0, 0, 125/108, -65/27, 125/54];
    beta(8,1:7) = [31/300, 0, 0, 0, 61/225, -2/9, 13/900];
    beta(9,1:8) = [2, 0, 0, -53/6, 704/45, -107/9, 67/90, 3];
    beta(10,1:9) = [-91/108, 0, 0, 23/108, -976/135, 311/54, -19/60, 17/6, -1/12];
    beta(11,1:10) = [2383/4100, 0, 0, -341/164, 4496/1025, -301/82, 2133/4100, 45/82, 45/162, 18/41];
    beta(12,1:11) = [3/205, 0, 0, 0, 0, -6/41, -3/205, -3/41, -3/41, 3/41, 6/41];
    beta(13,1:12) = [-1777/4100, 0, 0, -341/164, 4496/1025, -289/82, 2193/4100, 51/82, 33/164, 12/41, 0, 1];

    b7 = [41/840, 0, 0, 0, 0, 34/105, 9/35, 9/35, 9/280, 9/280, 41/840, 0, 0];

    R = R0; V = V0;
    for k = 2:N
        k_r = zeros(13, 3); k_v = zeros(13, 3);
        [k_r(1,:), k_v(1,:)] = stateequation(R, V);
        for i = 2:13
            R_temp = R + h * sum(beta(i, 1:i-1)' .* k_r(1:i-1, :), 1);
            V_temp = V + h * sum(beta(i, 1:i-1)' .* k_v(1:i-1, :), 1);
            [k_r(i,:), k_v(i,:)] = stateequation(R_temp, V_temp);
        end
        R = R + h * sum(b7' .* k_r, 1);
        V = V + h * sum(b7' .* k_v, 1);
        Xout_rkf78(k, :) = [R, V];
    end
    
    % --------------------------
    % 4. 误差计算 (关键修改部分)
    % --------------------------
    diff_vec = Xout_rk4 - Xout_rkf78;
    
    % 分量误差
    err_pos_x = diff_vec(:,1); err_pos_y = diff_vec(:,2); err_pos_z = diff_vec(:,3);
    err_vel_x = diff_vec(:,4); err_vel_y = diff_vec(:,5); err_vel_z = diff_vec(:,6);
    
    % 模长误差 (Magnitude Error)
    % 计算每一时刻的位置矢量差的模长 |r_rk4 - r_rkf78|
    err_pos_norm = sqrt(err_pos_x.^2 + err_pos_y.^2 + err_pos_z.^2);
    % 计算每一时刻的速度矢量差的模长 |v_rk4 - v_rkf78|
    err_vel_norm = sqrt(err_vel_x.^2 + err_vel_y.^2 + err_vel_z.^2);
    
    % --------------------------
    % 5. 绘图
    % --------------------------
    
    % --- Figure 1: 三维轨迹图 ---
    figure(1);
    set(gcf, 'Color', 'w');
    [X_sphere, Y_sphere, Z_sphere] = sphere(50);
    R_earth_disp = 6378.137;  
    
    % 关键修改：保存句柄并设置 DisplayName，以便 legend 识别
    h_earth = surf(X_sphere*R_earth_disp, Y_sphere*R_earth_disp, Z_sphere*R_earth_disp, ...
         'FaceColor', [0.2, 0.6, 1], 'EdgeColor', 'none', 'FaceAlpha', 0.5, ...
         'DisplayName', '地球'); % 设置显示名称
         
    axis equal; hold on; grid on;
    
    h_rk4 = plot3(Xout_rk4(:,1), Xout_rk4(:,2), Xout_rk4(:,3), 'k-', 'LineWidth', 1.5, 'DisplayName', 'RK4法');
    h_rkf = plot3(Xout_rkf78(:,1), Xout_rkf78(:,2), Xout_rkf78(:,3), 'g--', 'LineWidth', 1.5, 'DisplayName', 'RKF78法');
    
    xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
    title('卫星轨道三维轨迹对比');
    view(30, 30);
    
    % 显示图例，显式指定句柄，确保“地球”正确显示而不是 data1
    legend([h_earth, h_rk4, h_rkf], 'Location', 'best');
    hold off;
    
    % --- Figure 2: 位置误差图 (含模长) ---
    figure(2);
    set(gcf, 'Color', 'w', 'Name', '位置误差分析', 'Position', [100, 100, 1000, 700]);
    
    subplot(2, 2, 1);
    plot(tout, err_pos_x*1e-3, 'r'); title('X轴 位置误差'); ylabel('Error (km)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 2);
    plot(tout, err_pos_y*1e-3, 'b'); title('Y轴 位置误差'); ylabel('Error (km)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 3);
    plot(tout, err_pos_z*1e-3, 'k'); title('Z轴 位置误差'); xlabel('Time (s)'); ylabel('Error (km)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 4); % 新增模长误差绘图
    plot(tout, err_pos_norm*1e-3, 'm', 'LineWidth', 1.2); 
    title('位置矢量模长误差 |\Deltar|'); xlabel('Time (s)'); ylabel('Error (km)'); grid on; xlim([0 tf]);
    
    sgtitle('位置解算误差 (RK4 - RKF78)');
    
    % --- Figure 3: 速度误差图 (含模长) ---
    figure(3);
    set(gcf, 'Color', 'w', 'Name', '速度误差分析', 'Position', [150, 150, 1000, 700]);
    
    subplot(2, 2, 1);
    plot(tout, err_vel_x, 'r'); title('X轴 速度误差'); ylabel('Error (km/s)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 2);
    plot(tout, err_vel_y, 'b'); title('Y轴 速度误差'); ylabel('Error (km/s)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 3);
    plot(tout, err_vel_z, 'k'); title('Z轴 速度误差'); xlabel('Time (s)'); ylabel('Error (km/s)'); grid on; xlim([0 tf]);
    
    subplot(2, 2, 4); % 新增模长误差绘图
    plot(tout, err_vel_norm, 'm', 'LineWidth', 1.2); 
    title('速度矢量模长误差 |\Deltav|'); xlabel('Time (s)'); ylabel('Error (km/s)'); grid on; xlim([0 tf]);
    
    sgtitle('速度解算误差 (RK4 - RKF78)');

end

% =========================================================
% 局部函数
% =========================================================

function [Rdot, Vdot] = stateequation(R, V)
    global mu
    r = norm(R);
    Rdot = V;
    Vdot = -mu / (r^3) * R;
end

function result = coe_caculate(R, V)
    global mu; 
    eps = 1e-10;
    r = norm(R); v = norm(V); vr = dot(R,V)/r;
    H = cross(R,V); h = norm(H);
    incl = acos(H(3)/h);
    N_vec = cross([0 0 1], H); n = norm(N_vec);
    if n ~= 0
        RA = acos(N_vec(1)/n);
        if N_vec(2) < 0, RA = 2*pi - RA; end
    else, RA = 0; end
    E_vec = (1/mu)*((v^2 - mu/r)*R - r*vr*V); e = norm(E_vec);
    if n ~= 0
        if e > eps
            w = acos(dot(N_vec, E_vec)/(n*e));
            if E_vec(3) < 0, w = 2*pi - w; end
        else, w = 0; end
    else, w = 0; end
    if e > eps
        TA = acos(dot(E_vec, R)/(e*r));
        if vr < 0, TA = 2*pi - TA; end
    else
        cp = cross(N_vec, R);
        if cp(3) >= 0, TA = acos(dot(N_vec, R)/(n*r));
        else, TA = 2*pi - acos(dot(N_vec, R)/(n*r)); end
    end
    a = h^2 / (mu*(1 - e^2));
    result = [h e RA incl w TA a];
    fprintf('------- 轨道信息 ----------\n');
    fprintf('半长轴 a (km) = %g\n', a);
    fprintf('偏心率 e      = %g\n', e);
end