clc; clear; close all;

%% 1. 参数与初始化
global mu 
mu = 398600.4418;       % 地球引力常数
Re = 6371;              % 地球半径

% 初始状态
R0 = [-5102, -8228, -2105];  
V0 = [-4.348, 3.478, -2.846]; 

% 简单计算周期 (仅预估仿真时间)
r_mag = norm(R0); v_mag = norm(V0);
E = v_mag^2/2 - mu/r_mag;
a = -mu / (2*E);
T_period = 2*pi*sqrt(a^3/mu);

h = 10;                 % 步长设为 10s (0.1s太慢且对于RKF78过于保守)
tf = T_period * 1.5;    % 仿真 1.5 个周期
tout = 0:h:tf;
N = length(tout);

%% 2. RK4 求解
Xout_rk4 = zeros(N,6);
R = R0; V = V0;
Xout_rk4(1,:) = [R, V];

fprintf('正在进行 RK4 计算...\n');
for k = 2:N
    [k1r, k1v] = stateequation(R, V);
    [k2r, k2v] = stateequation(R + 0.5*h*k1r, V + 0.5*h*k1v);
    [k3r, k3v] = stateequation(R + 0.5*h*k2r, V + 0.5*h*k2v);
    [k4r, k4v] = stateequation(R + h*k3r, V + h*k3v);
    
    R = R + h*(k1r + 2*k2r + 2*k3r + k4r)/6;
    V = V + h*(k1v + 2*k2v + 2*k3v + k4v)/6;
    Xout_rk4(k,:) = [R, V];
end

%% 3. RKF7(8) 求解 (修正了 beta 矩阵)
Xout_rkf78 = zeros(N, 6);  
Xout_rkf78(1,:) = [R0, V0]; 

% RKF7(8) 完整系数 (根据之前的图片修正)
beta = zeros(13,13);
beta(2,1)=2/27;
beta(3,1)=1/36; beta(3,2)=1/12;
beta(4,1)=1/24; beta(4,3)=1/8;
beta(5,1)=5/12; beta(5,3)=-25/16; beta(5,4)=25/16;
beta(6,1)=1/20; beta(6,4)=1/4; beta(6,5)=1/5;
beta(7,1)=-25/108; beta(7,4)=125/108; beta(7,5)=-65/27; beta(7,6)=125/54;
beta(8,1)=31/300; beta(8,5)=61/225; beta(8,6)=-2/9; beta(8,7)=13/900;
beta(9,1)=2; beta(9,4)=-53/6; beta(9,5)=704/45; beta(9,6)=-107/9; beta(9,7)=67/90; beta(9,8)=3;
beta(10,1)=-91/108; beta(10,4)=23/108; beta(10,5)=-976/135; beta(10,6)=311/54; beta(10,7)=-19/60; beta(10,8)=17/6; beta(10,9)=-1/12;
beta(11,1)=2383/4100; beta(11,4)=-341/164; beta(11,5)=4496/1025; beta(11,6)=-301/82; beta(11,7)=2133/4100; beta(11,8)=45/82; beta(11,9)=45/162; beta(11,10)=18/41;
beta(12,1)=3/205; beta(12,6)=-6/41; beta(12,7)=-3/205; beta(12,8)=-3/41; beta(12,9)=3/41; beta(12,10)=6/41;
beta(13,1)=-1777/4100; beta(13,4)=-341/164; beta(13,5)=4496/1025; beta(13,6)=-289/82; beta(13,7)=2193/4100; beta(13,8)=51/82; beta(13,9)=33/164; beta(13,10)=12/41; beta(13,12)=1;

% 8阶解权重 (用于高精度输出)
c_8th = [41/840, 0, 0, 0, 0, 34/105, 9/35, 9/35, 9/280, 9/280, 41/840, 0, 41/840];

R = R0; V = V0;

fprintf('正在进行 RKF7(8) 计算...\n');
for k = 2:N
    k_r = zeros(13, 3);
    k_v = zeros(13, 3);
    
    % Stage 1
    [k_r(1,:), k_v(1,:)] = stateequation(R, V);
    
    % Stage 2-13
    for i = 2:13
        % 计算中间状态 (Vectorized sum)
        % 注意：beta(i, 1:i-1) 是 1x(i-1)，k_r(1:i-1, :) 是 (i-1)x3
        % 乘积需要转置 beta 以匹配矩阵乘法，或者用循环
        sum_r = zeros(1,3); sum_v = zeros(1,3);
        for j = 1:i-1
            if beta(i,j) ~= 0
                sum_r = sum_r + beta(i,j) * k_r(j,:);
                sum_v = sum_v + beta(i,j) * k_v(j,:);
            end
        end
        
        [k_r(i,:), k_v(i,:)] = stateequation(R + h*sum_r, V + h*sum_v);
    end
    
    % 输出更新 (使用8阶权重)
    delta_R = zeros(1,3); delta_V = zeros(1,3);
    for j = 1:13
        if c_8th(j) ~= 0
            delta_R = delta_R + c_8th(j) * k_r(j,:);
            delta_V = delta_V + c_8th(j) * k_v(j,:);
        end
    end
    
    R = R + h * delta_R;
    V = V + h * delta_V;
    
    Xout_rkf78(k,:) = [R, V];
end

%% 4. 绘图与误差分析
% 计算误差 (RK4 - RKF78)
error_vec = Xout_rk4 - Xout_rkf78;
error_pos = error_vec(:, 1:3);

figure(1);
plot3(Xout_rk4(:,1), Xout_rk4(:,2), Xout_rk4(:,3), 'r--', 'LineWidth', 1.5); hold on;
plot3(Xout_rkf78(:,1), Xout_rkf78(:,2), Xout_rkf78(:,3), 'g-', 'LineWidth', 1);
[X,Y,Z] = sphere(30); surf(X*Re, Y*Re, Z*Re, 'FaceColor','b', 'EdgeColor','none', 'FaceAlpha',0.3);
axis equal; grid on; legend('RK4', 'RKF7(8)', 'Earth'); title('轨迹对比');

figure(2);
subplot(3,1,1); plot(tout, error_pos(:,1), 'r'); ylabel('\Delta X (km)'); title('RK4 相对于 RKF7(8) 的位置误差 (X轴)'); grid on;
subplot(3,1,2); plot(tout, error_pos(:,2), 'b'); ylabel('\Delta Y (km)'); grid on;
subplot(3,1,3); plot(tout, error_pos(:,3), 'k'); ylabel('\Delta Z (km)'); grid on; xlabel('Time (s)');

%% 局部函数
function [dr, dv] = stateequation(r_vec, v_vec)
    global mu
    r = norm(r_vec);
    dv = -mu * r_vec / r^3;
    dr = v_vec;
end