clear; clc; close all;

%% 1. 参数设置
global mu
mu = 398600.4418;       % 地球引力常数 (km^3/s^2)
Re = 6371;              % 地球半径 (km)

% 初始状态
r0 = [-5102; -8228; -2105];   % km
v0 = [-4.348; 3.478; -2.846]; % km/s
y0 = [r0; v0];

% 仿真时间设置
h = 0.1;                 % 积分步长 10秒
t_span = [0, 15000];    % 仿真时长 (秒)

%% 2. 数值积分求解
% RK4 求解
[t_rk4, y_rk4] = solve_ode_custom(@sys_model, t_span, y0, h, 'rk4');

% RKF7(8) 求解 
[t_rkf, y_rkf] = solve_ode_custom(@sys_model, t_span, y0, h, 'rkf78');

%% 3. 数据处理与误差计算
% 计算 r 和 v 的模长
r_norm_rk4 = sqrt(sum(y_rk4(:,1:3).^2, 2)); 
v_norm_rk4 = sqrt(sum(y_rk4(:,4:6).^2, 2));

r_norm_rkf = sqrt(sum(y_rkf(:,1:3).^2, 2)); 
v_norm_rkf = sqrt(sum(y_rkf(:,4:6).^2, 2));

delta_y = y_rk4 - y_rkf; % 包含 [dx, dy, dz, dvx, dvy, dvz]

% 提取分量差值
diff_pos = delta_y(:, 1:3); % [Delta_x, Delta_y, Delta_z]
diff_vel = delta_y(:, 4:6); % [Delta_vx, Delta_vy, Delta_vz]

% 计算差值的模长 
err_r_norm = sqrt(sum(diff_pos.^2, 2)); % |Delta r|
err_v_norm = sqrt(sum(diff_vel.^2, 2)); % |Delta v|

%% 4. 绘图结果

% --- 三维轨迹对比 ---
figure(1); set(gcf, 'Color', 'w', 'Name', 'Figure 1: 3D Trajectory Comparison');
plot3(y_rk4(:,1), y_rk4(:,2), y_rk4(:,3), 'r--', 'LineWidth', 1.5); hold on;
plot3(y_rkf(:,1), y_rkf(:,2), y_rkf(:,3), 'g-', 'LineWidth', 1.5);
% 画地球示意
[X_e,Y_e,Z_e] = sphere(30);
surf(X_e*Re, Y_e*Re, Z_e*Re, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.3, 'HandleVisibility', 'off');
plot3(0, 0, 0, 'Color', 'blue', 'LineWidth', 10, 'HandleVisibility', 'on', 'DisplayName', '地球');
grid on; axis equal; view(3);
xlabel('X (km)'); ylabel('Y (km)'); zlabel('Z (km)');
legend('RK4', 'RKF7(8)', '地球');
title('轨迹对比 (RK4 vs RKF7(8))');

% --- 模长误差图 ---
figure(2); set(gcf, 'Color', 'w', 'Name', 'Figure 6: Magnitude Error Analysis');

subplot(2,1,1);
plot(t_rk4, err_r_norm*1e-6, 'k-', 'LineWidth', 1.5);
ylabel('位置误差模长 |\Delta r| (km)'); % 明确单位 km
xlabel('时间 t (s)');
title('RK4 相对于 RKF7(8) 的位置误差模长');
grid on;

subplot(2,1,2);
plot(t_rk4, err_v_norm*1e-3, 'k-', 'LineWidth', 1.5);
ylabel('速度误差模长 |\Delta v| (km/s)'); % 明确单位 km/s
xlabel('时间 t (s)');
title('RK4 相对于 RKF7(8) 的速度误差模长');
grid on;

% ---  (x,y,z, vx,vy,vz) ---
figure(3); set(gcf, 'Color', 'w', 'Name', 'Figure 7: Component Error Analysis');
titles_err_pos = {'\Delta x', '\Delta y', '\Delta z'};
titles_err_vel = {'\Delta v_x', '\Delta v_y', '\Delta v_z'};

for i = 1:3
    % 左列：位置分量误差
    subplot(3, 2, 2*i - 1);
    plot(t_rk4, diff_pos(:, i)*1e-3, 'r-', 'LineWidth', 1.2);
    ylabel([titles_err_pos{i} ' (km)']); % 单位 km
    grid on;
    if i == 1, title('位置分量误差 (km)'); end
    if i == 3, xlabel('时间 t (s)'); end
    
    % 右列：速度分量误差
    subplot(3, 2, 2*i);
    plot(t_rk4, diff_vel(:, i)*1e-3, 'b-', 'LineWidth', 1.2);
    ylabel([titles_err_vel{i} ' (km/s)']); % 单位 km/s
    grid on;
    if i == 1, title('速度分量误差 (km/s)'); end
    if i == 3, xlabel('时间 t (s)'); end
end
sgtitle('RK4 与 RKF7(8) 各状态分量差值随时间变化');

%% --- 局部函数定义 ---

% 二体动力学模型
function dydt = sys_model(~, y)
    global mu
    r_vec = y(1:3);
    v_vec = y(4:6);
    r = norm(r_vec);
    a_vec = -mu * r_vec / r^3;
    dydt = [v_vec; a_vec];
end

% 通用求解器框架
function [t, y] = solve_ode_custom(fun, t_span, y0, h, method)
    t = t_span(1):h:t_span(2);
    y = zeros(length(t), length(y0));
    y(1, :) = y0';
    
    switch method
        case 'rk4'
            for i = 1:length(t)-1
                curr_t = t(i);
                curr_y = y(i, :)';
                k1 = fun(curr_t, curr_y);
                k2 = fun(curr_t + 0.5*h, curr_y + 0.5*h*k1);
                k3 = fun(curr_t + 0.5*h, curr_y + 0.5*h*k2);
                k4 = fun(curr_t + h, curr_y + h*k3);
                y(i+1, :) = y(i, :)' + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
            end
            
        case 'rkf78'
            [alpha, beta, c] = get_rkf78_coeffs();
            stages = 13;
            for i = 1:length(t)-1
                curr_t = t(i);
                curr_y = y(i, :)';
                K = zeros(length(y0), stages); 
                K(:, 1) = fun(curr_t, curr_y);
                for j = 1:stages-1
                    temp_sum = zeros(length(y0), 1);
                    for m = 0:j-1
                         if beta(j+1, m+1) ~= 0
                            temp_sum = temp_sum + beta(j+1, m+1) * K(:, m+1);
                         end
                    end
                    K(:, j+1) = fun(curr_t + alpha(j+1)*h, curr_y + h*temp_sum);
                end
                final_sum = zeros(length(y0), 1);
                for j = 0:stages-1
                    if c(j+1) ~= 0
                        final_sum = final_sum + c(j+1) * K(:, j+1);
                    end
                end
                y(i+1, :) = curr_y + h * final_sum;
            end
    end
end

% RKF7(8) 系数定义
function [alpha, beta, c] = get_rkf78_coeffs()
    alpha = zeros(13, 1);
    beta = zeros(13, 13);
    c = zeros(13, 1);
    
    % Alpha
    A_vals = {0, 2/27, 1/9, 1/6, 5/12, 1/2, 5/6, 1/6, 2/3, 1/3, 1, 0, 1};
    for k=1:13, alpha(k) = A_vals{k}; end
    
    % Beta (Row 2 to 13)
    beta(2,1) = 2/27;
    beta(3,1) = 1/36; beta(3,2) = 1/12;
    beta(4,1) = 1/24; beta(4,2) = 0; beta(4,3) = 1/8;
    beta(5,1) = 5/12; beta(5,2) = 0; beta(5,3) = -25/16; beta(5,4) = 25/16;
    beta(6,1) = 1/20; beta(6,2) = 0; beta(6,3) = 0; beta(6,4) = 1/4; beta(6,5) = 1/5;
    beta(7,1) = -25/108; beta(7,2) = 0; beta(7,3) = 0; beta(7,4) = 125/108; beta(7,5) = -65/27; beta(7,6) = 125/54;
    beta(8,1) = 31/300; beta(8,2) = 0; beta(8,3) = 0; beta(8,4) = 0; beta(8,5) = 61/225; beta(8,6) = -2/9; beta(8,7) = 13/900;
    beta(9,1) = 2; beta(9,2) = 0; beta(9,3) = 0; beta(9,4) = -53/6; beta(9,5) = 704/45; beta(9,6) = -107/9; beta(9,7) = 67/90; beta(9,8) = 3;
    beta(10,1) = -91/108; beta(10,2) = 0; beta(10,3) = 0; beta(10,4) = 23/108; beta(10,5) = -976/135; beta(10,6) = 311/54; beta(10,7) = -19/60; beta(10,8) = 17/6; beta(10,9) = -1/12;
    beta(11,1) = 2383/4100; beta(11,2) = 0; beta(11,3) = 0; beta(11,4) = -341/164; beta(11,5) = 4496/1025; beta(11,6) = -301/82; beta(11,7) = 2133/4100; beta(11,8) = 45/82; beta(11,9) = 45/162; beta(11,10) = 18/41;
    beta(12,1) = 3/205; beta(12,2) = 0; beta(12,3) = 0; beta(12,4) = 0; beta(12,5) = 0; beta(12,6) = -6/41; beta(12,7) = -3/205; beta(12,8) = -3/41; beta(12,9) = 3/41; beta(12,10) = 6/41; beta(12,11) = 0;
    beta(13,1) = -1777/4100; beta(13,2) = 0; beta(13,3) = 0; beta(13,4) = -341/164; beta(13,5) = 4496/1025; beta(13,6) = -289/82; beta(13,7) = 2193/4100; beta(13,8) = 51/82; beta(13,9) = 33/164; beta(13,10) = 12/41; beta(13,11) = 0; beta(13,12) = 1;
    
    % C (Weights)
    C_vals = {41/840, 0, 0, 0, 0, 34/105, 9/35, 9/35, 9/280, 9/280, 41/840, 0, 41/840};
    for k=1:13, c(k) = C_vals{k}; end
end