function iod_gauss_experiment()
    % ---------------------------------------------------------------------
    % 航天器轨道动力学实践教程 - 第6章 初始轨道确定实验
    % ---------------------------------------------------------------------
    clc; clear; close all;
    format long g;

    % --- 1. 物理常数与观测数据 ---
    mu = 398600.4418;       % 地球引力常数 km^3/s^2 [cite: 44]
    Re = 6378.137;          % 地球赤道半径 km
    f_earth = 1/298.257;    % 地球扁率
    deg2rad = pi/180;
    
    % 观测站位置 (北纬60度, 高度500m) [cite: 212]
    phi_gd = 60.0 * deg2rad; 
    H_station = 0.5;        % km (500m)

    % 观测数据表 [cite: 213]
    % t(min), theta(deg), alpha(deg), delta(deg)
    obs_data = [
        0.0,  150.000, 157.783, 24.2403;
        5.0,  151.253, 159.221, 27.2993;
        10.0, 152.507, 160.526, 29.8982
    ];

    t_min = obs_data(:,1);
    theta_deg = obs_data(:,2);
    alpha_deg = obs_data(:,3);
    delta_deg = obs_data(:,4);

    % --- 2. 数据预处理 ---
    
    % 时间间隔 (秒)
    t_sec = t_min * 60;
    tau1 = t_sec(1) - t_sec(2); % t1 - t2 [cite: 85]
    tau3 = t_sec(3) - t_sec(2); % t3 - t2 [cite: 85]
    tau = t_sec(3) - t_sec(1);  % t3 - t1 [cite: 101]

    % 计算观测站位置矢量 R (ECI坐标系) [cite: 27]
    R_vec = zeros(3,3);
    for i = 1:3
        R_vec(:,i) = calc_site_position(phi_gd, H_station, theta_deg(i)*deg2rad, Re, f_earth);
    end

    % 计算视线方向单位矢量 L (rho_hat) [cite: 25]
    L = zeros(3,3);
    for i = 1:3
        ra = alpha_deg(i) * deg2rad;
        dec = delta_deg(i) * deg2rad;
        L(:,i) = [cos(dec)*cos(ra); cos(dec)*sin(ra); sin(dec)];
    end

    % --- 3. 任务(1): 高斯法初始计算 (无迭代) ---
    fprintf('================ 任务(1) 高斯法初始结果 ================\n');
    
    % 计算几何常数 D0 [cite: 135]
    D0 = dot(L(:,1), cross(L(:,2), L(:,3)));
    
    % 计算 Dij [cite: 140, 150, 152]
    D = zeros(3,4); % 这里的索引对应 D(i,j) -> Dij
    % 注意：为了代码方便，D矩阵行对应观测次数1,2,3，列对应公式中的下标
    % 公式中 D11, D21, D31 对应 R1, R2, R3 点乘 (L2 x L3)
    
    % 第一组 (L2 x L3)
    cross23 = cross(L(:,2), L(:,3));
    D(1,1) = dot(R_vec(:,1), cross23); % D11
    D(2,1) = dot(R_vec(:,2), cross23); % D21
    D(3,1) = dot(R_vec(:,3), cross23); % D31
    
    % 第二组 (L1 x L3)
    cross13 = cross(L(:,1), L(:,3));
    D(1,2) = dot(R_vec(:,1), cross13); % D12
    D(2,2) = dot(R_vec(:,2), cross13); % D22
    D(3,2) = dot(R_vec(:,3), cross13); % D32
    
    % 第三组 (L1 x L2)
    cross12 = cross(L(:,1), L(:,2));
    D(1,3) = dot(R_vec(:,1), cross12); % D13
    D(2,3) = dot(R_vec(:,2), cross12); % D23
    D(3,3) = dot(R_vec(:,3), cross12); % D33
    
    % 计算 A 和 B [cite: 160, 161]
    A = (1/D0) * (-D(1,2)*(tau3/tau) + D(2,2) + D(3,2)*(tau1/tau));
    B = (1/(6*D0)) * (D(1,2)*(tau3^2 - tau^2)*(tau3/tau) + D(3,2)*(tau^2 - tau1^2)*(tau1/tau));
    
    % 计算 E [cite: 178]
    E_dot = dot(R_vec(:,2), L(:,2));
    R2_sq = dot(R_vec(:,2), R_vec(:,2));
    
    % 求解 r2 的八次多项式 [cite: 184, 187]
    % x^8 + a*x^6 + b*x^3 + c = 0
    coef_a = -(A^2 + 2*A*E_dot + R2_sq);
    coef_b = -2 * mu * B * (A + E_dot);
    coef_c = -mu^2 * B^2;
    
    % MATLAB roots 求解多项式根: [1, 0, a, 0, 0, b, 0, 0, c]
    poly_coeffs = [1, 0, coef_a, 0, 0, coef_b, 0, 0, coef_c];
    r2_roots = roots(poly_coeffs);
    
    % 筛选合理的实数根 (r > Re)
    r2_est = 0;
    for k = 1:length(r2_roots)
        if isreal(r2_roots(k)) && r2_roots(k) > Re
            r2_est = real(r2_roots(k));
            break; 
        end
    end
    fprintf('估算的 r2 (km): %.6f\n', r2_est);
    
    % 计算近似系数 u
    u = mu / r2_est^3;
    
    % 计算 Gauss 系数 c1, c3 的近似值 [cite: 113, 115]
    % 注意：PDF公式 6.20a/b
    c1_approx = (tau3/tau) * (1 + (1/6)*u*(tau^2 - tau3^2));
    c3_approx = -(tau1/tau) * (1 + (1/6)*u*(tau^2 - tau1^2));
    
    % 计算斜距 rho1, rho2, rho3
    % 使用通用解公式 Eq 6.24a, 6.25a, 6.26a
    rho1 = (1/D0) * (-D(1,1) + (1/c1_approx)*D(2,1) - (c3_approx/c1_approx)*D(3,1));
    rho2 = (1/D0) * (-c1_approx*D(1,2) + D(2,2) - c3_approx*D(3,2));
    rho3 = (1/D0) * (-(c1_approx/c3_approx)*D(1,3) + (1/c3_approx)*D(2,3) - D(3,3));
    
    % 计算位置矢量 r1, r2, r3 [cite: 25]
    r_vec = zeros(3,3);
    r_vec(:,1) = R_vec(:,1) + rho1 * L(:,1);
    r_vec(:,2) = R_vec(:,2) + rho2 * L(:,2);
    r_vec(:,3) = R_vec(:,3) + rho3 * L(:,3);
    
    % 计算速度矢量 v2 [cite: 36, 201]
    % 需要 Lagrange 系数 f1, g1, f3, g3 的级数近似 [cite: 86-89]
    f1 = 1 - 0.5 * u * tau1^2;
    f3 = 1 - 0.5 * u * tau3^2;
    g1 = tau1 - (1/6) * u * tau1^3;
    g3 = tau3 - (1/6) * u * tau3^3;
    
    v2_est = (1 / (f1*g3 - f3*g1)) * (-f3 * r_vec(:,1) + f1 * r_vec(:,3));
    
    fprintf('初始位置 r2: [%.4f, %.4f, %.4f] km\n', r_vec(:,2));
    fprintf('初始速度 v2: [%.4f, %.4f, %.4f] km/s\n', v2_est);

    % --- 4. 任务(2): 迭代改进 ---
    fprintf('\n================ 任务(2) 迭代改进结果 ================\n');
    
    % 设置迭代变量
    r2_iter = r_vec(:,2);
    v2_iter = v2_est;
    tol = 1e-8;
    max_iter = 20;
    
    for iter = 1:max_iter
        r2_mag = norm(r2_iter);
        v2_mag = norm(v2_iter);
        alpha = 2/r2_mag - v2_mag^2/mu; % 轨道能量参数
        
        % 1. 使用通用变量法计算精确的 f 和 g [cite: 205]
        [f1_ex, g1_ex] = universal_variable_fg(r2_iter, v2_iter, tau1, mu);
        [f3_ex, g3_ex] = universal_variable_fg(r2_iter, v2_iter, tau3, mu);
        
        % 2. 更新 c1, c3 [cite: 80, 81]
        denom = f1_ex*g3_ex - f3_ex*g1_ex;
        c1_new = g3_ex / denom;
        c3_new = -g1_ex / denom;
        
        % 3. 重新计算 rho (利用线性方程组关系) [cite: 24, 25, 26]
        % 使用更新后的 c1, c3 重新代入通用公式求解
        rho1_new = (1/D0) * (-D(1,1) + (1/c1_new)*D(2,1) - (c3_new/c1_new)*D(3,1));
        rho2_new = (1/D0) * (-c1_new*D(1,2) + D(2,2) - c3_new*D(3,2));
        rho3_new = (1/D0) * (-(c1_new/c3_new)*D(1,3) + (1/c3_new)*D(2,3) - D(3,3));
        
        % 4. 更新 r 矢量
        r1_new = R_vec(:,1) + rho1_new * L(:,1);
        r2_new = R_vec(:,2) + rho2_new * L(:,2);
        r3_new = R_vec(:,3) + rho3_new * L(:,3);
        
        % 5. 更新 v2 [cite: 201]
        v2_new = (1 / denom) * (-f3_ex * r1_new + f1_ex * r3_new);
        
        % 检查收敛性
        diff = norm(r2_new - r2_iter);
        r2_iter = r2_new;
        v2_iter = v2_new;
        
        fprintf('迭代 %d: r2 误差 = %.2e km\n', iter, diff);
        if diff < tol
            break;
        end
    end
    
    fprintf('修正后位置 r2: [%.4f, %.4f, %.4f] km\n', r2_iter);
    fprintf('修正后速度 v2: [%.4f, %.4f, %.4f] km/s\n', v2_iter);

    % --- 5. 任务(3): 计算轨道根数 ---
    fprintf('\n================ 任务(3) 轨道根数计算 ================\n');
    coe = rv2coe(r2_iter, v2_iter, mu);
    
    fprintf('半长轴 (a):     %.4f km\n', coe(1));
    fprintf('偏心率 (e):     %.6f\n', coe(2));
    fprintf('轨道倾角 (i):   %.4f deg\n', coe(3));
    fprintf('升交点赤经 (Ω): %.4f deg\n', coe(4));
    fprintf('近地点幅角 (ω): %.4f deg\n', coe(5));
    fprintf('真近点角 (ν):   %.4f deg\n', coe(6));

end

% ---------------------------------------------------------------------
% 辅助函数
% ---------------------------------------------------------------------

function R = calc_site_position(phi, H, lst, Re, f)
    % 根据纬度、高度、恒星时计算测站位置 [cite: 27]
    % 输入: phi(rad), H(km), lst(rad)
    e2 = 2*f - f^2;
    sin_phi = sin(phi);
    N = Re / sqrt(1 - e2 * sin_phi^2);
    
    x = (N + H) * cos(phi) * cos(lst);
    y = (N + H) * cos(phi) * sin(lst);
    z = (N * (1 - e2) + H) * sin_phi;
    R = [x; y; z];
end

function [f, g] = universal_variable_fg(r_vec, v_vec, dt, mu)
    % 通用变量法求精确的 Lagrange 系数 f 和 g [cite: 39]
    r = norm(r_vec);
    v = norm(v_vec);
    vr = dot(r_vec, v_vec) / r;
    alpha = 2/r - v^2/mu;
    
    % 初始猜测 chi
    chi = sqrt(mu) * abs(alpha) * dt;
    if alpha > 1e-6
        chi = chi / sqrt(alpha);
    end
    
    % Newton 迭代求解 chi
    ratio = 1;
    max_it = 100;
    it = 0;
    while abs(ratio) > 1e-8 && it < max_it
        z = alpha * chi^2;
        [S, C] = stumpff(z);
        
        r_val = (chi^2)*C + r*(1 - alpha*chi^2*S) + vr*chi*(1 - z*S);
        t_val = (chi^3*S + r*chi*(1 - alpha*chi^2*S) + vr*chi^2*C) / sqrt(mu);
        
        ratio = (t_val - dt) / r_val;
        chi = chi - ratio;
        it = it + 1;
    end
    
    z = alpha * chi^2;
    [S, C] = stumpff(z);
    
    f = 1 - (chi^2 / r) * C;
    g = dt - (chi^3 / sqrt(mu)) * S;
end

function [S, C] = stumpff(z)
    % Stumpff 函数
    if z > 1e-6
        S = (sqrt(z) - sin(sqrt(z))) / (sqrt(z))^3;
        C = (1 - cos(sqrt(z))) / z;
    elseif z < -1e-6
        S = (sinh(sqrt(-z)) - sqrt(-z)) / (sqrt(-z))^3;
        C = (cosh(sqrt(-z)) - 1) / (-z);
    else
        S = 1/6;
        C = 1/2;
    end
end

function coe = rv2coe(r_vec, v_vec, mu)
    % 状态矢量转轨道根数
    r = norm(r_vec);
    v = norm(v_vec);
    h_vec = cross(r_vec, v_vec);
    h = norm(h_vec);
    
    % 节点矢量
    n_vec = cross([0;0;1], h_vec);
    n = norm(n_vec);
    
    % 偏心率矢量
    e_vec = (1/mu) * ((v^2 - mu/r)*r_vec - dot(r_vec, v_vec)*v_vec);
    e = norm(e_vec);
    
    % 能量与半长轴
    xi = v^2/2 - mu/r;
    a = -mu / (2*xi);
    
    % 倾角
    inc = acos(h_vec(3)/h) * 180/pi;
    
    % 升交点赤经
    if n ~= 0
        Omega = acos(n_vec(1)/n) * 180/pi;
        if n_vec(2) < 0, Omega = 360 - Omega; end
    else
        Omega = 0;
    end
    
    % 近地点幅角
    if n~=0 && e~=0
        w = acos(dot(n_vec, e_vec)/(n*e)) * 180/pi;
        if e_vec(3) < 0, w = 360 - w; end
    else
        w = 0;
    end
    
    % 真近点角
    if e~=0
        nu = acos(dot(e_vec, r_vec)/(e*r)) * 180/pi;
        if dot(r_vec, v_vec) < 0, nu = 360 - nu; end
    else
        nu = 0;
    end
    
    coe = [a, e, inc, Omega, w, nu];
end