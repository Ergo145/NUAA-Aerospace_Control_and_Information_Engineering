function flight_simulation()
    % 飞行器参数
    m = 1000;           % 质量 kg
    I_x = 1000;         % 转动惯量 kg·m²
    I_y = 2000;
    I_z = 1500;
    g = 9.81;           % 重力加速度 m/s²
    omega_z = 7.292e-5; % 地球自转角速度 rad/s
    
    % 初始条件 (根据题目要求初始值为30)
    R0 = 6371000 + 30000;  % 地球半径 + 30km高度
    tau0 = 0;              % 经度
    delta0 = pi/4;         % 纬度
    V0 = 300;              % 速度 m/s
    gamma0 = 0;            % 航迹角
    z0 = 0;                % 航向角
    alpha0 = 0.1;          % 迎角
    beta0 = 0;             % 侧滑角
    sigma0 = 0;            % 滚转角
    p0 = 0; q0 = 0; r0 = 0;% 角速率
    
    x0 = [R0; tau0; delta0; V0; gamma0; z0; alpha0; beta0; sigma0; p0; q0; r0];
    
    % 仿真时间
    tspan = [0 1000];
    
    % 求解ODE
    [t, x] = ode45(@(t,x) flight_dynamics(t,x,m,I_x,I_y,I_z,g,omega_z), tspan, x0);
    
    % 提取结果
    R = x(:,1); tau = x(:,2); delta = x(:,3);
    V = x(:,4); gamma = x(:,5); z_angle = x(:,6);
    alpha = x(:,7); beta = x(:,8); sigma = x(:,9);
    p = x(:,10); q = x(:,11); r = x(:,12);
    
    % 绘制结果
    plot_results(t, x);
end

function dxdt = flight_dynamics(t, x, m, I_x, I_y, I_z, g, omega_z)
    % 提取状态变量
    R = x(1); tau = x(2); delta = x(3);
    V = x(4); gamma = x(5); z = x(6);
    alpha = x(7); beta = x(8); sigma = x(9);
    p = x(10); q = x(11); r = x(12);
    
    % 气动力和力矩计算 (需要根据具体飞行器模型补充)
    [L, D, C, m_xz, m_yz, n_xz] = calculate_aerodynamics(alpha, beta, V, R);
    
    % 位置方程组
    dR_dt = V * sin(gamma);
    dtau_dt = (V * cos(gamma) * sin(z)) / (R * cos(delta));
    ddelta_dt = (V * cos(gamma) * cos(z)) / R;
    
    % 速度方程组
    dV_dt = D/m - g*sin(gamma) + omega_z^2 * R * cos(delta) * ...
            (sin(gamma)*cos(delta) - cos(gamma)*sin(delta)*cos(z));
    
    dgamma_dt = (L*cos(sigma) + C*sin(sigma))/(m*V) + (V/R - g/V)*cos(gamma) + ...
                2*omega_z*cos(delta)*sin(z) + ...
                (omega_z^2 * R * cos(delta) * (cos(delta)*cos(gamma) + ...
                sin(delta)*sin(gamma)*cos(z)))/V;
    
    dz_dt = -(L*sin(sigma) - C*cos(sigma))/(m*V*cos(gamma)) + ...
            (V/R)*cos(gamma)*sin(z)*tan(delta) + ...
            (2*omega_z*(sin(delta) - cos(delta)*tan(gamma)*cos(z)) + ...
            omega_z^2 * R * sin(delta)*cos(delta)*sin(z))/(V*cos(gamma));
    
    % 气流姿态角方程组 (简化版本)
    dalpha_dt = q - (p*cos(alpha) + r*sin(alpha))*tan(beta);
    dbeta_dt = p*sin(alpha) - r*cos(alpha);
    dsigma_dt = -p*cos(alpha)*cos(beta) - q*sin(beta) - r*sin(alpha)*cos(beta);
    
    % 角速率方程组
    dp_dt = (I_y - I_z)*q*r/I_x + m_xz/I_x;
    dq_dt = (I_z - I_x)*p*r/I_y + m_yz/I_y;
    dr_dt = (I_x - I_y)*p*q/I_z + n_xz/I_z;
    
    dxdt = [dR_dt; dtau_dt; ddelta_dt; dV_dt; dgamma_dt; dz_dt; ...
            dalpha_dt; dbeta_dt; dsigma_dt; dp_dt; dq_dt; dr_dt];
end

function [L, D, C, m_xz, m_yz, n_xz] = calculate_aerodynamics(alpha, beta, V, R)
    % 简化气动力计算 (需要根据具体飞行器模型完善)
    rho = 1.225 * exp(-(R-6371000)/8000); % 大气密度近似
    q_bar = 0.5 * rho * V^2;              % 动压
    S = 50;                               % 参考面积 m²
    
    % 简化气动力系数
    C_L = 2*pi*alpha;
    C_D = 0.02 + 0.1*alpha^2;
    C_C = -0.1*beta;
    
    L = q_bar * S * C_L;
    D = q_bar * S * C_D;
    C = q_bar * S * C_C;
    
    % 简化力矩系数
    m_xz = 0; m_yz = 0; n_xz = 0;
end

function plot_results(t, x)
    figure;
    
    % 位置轨迹
    subplot(3,2,1);
    plot(t, x(:,1)-6371000); % 高度
    ylabel('高度 (m)');
    title('飞行高度');
    grid on;
    
    subplot(3,2,2);
    plot(x(:,2), x(:,3)); % 经纬度
    xlabel('经度 (rad)');
    ylabel('纬度 (rad)');
    title('水平轨迹');
    grid on;
    
    % 速度状态
    subplot(3,2,3);
    plot(t, x(:,4));
    ylabel('速度 (m/s)');
    title('飞行速度');
    grid on;
    
    subplot(3,2,4);
    plot(t, rad2deg(x(:,5)), t, rad2deg(x(:,6)));
    ylabel('角度 (deg)');
    title('航迹角和航向角');
    legend('γ', 'z');
    grid on;
    
    % 姿态角
    subplot(3,2,5);
    plot(t, rad2deg(x(:,7)), t, rad2deg(x(:,8)), t, rad2deg(x(:,9)));
    ylabel('角度 (deg)');
    xlabel('时间 (s)');
    title('姿态角');
    legend('α', 'β', 'σ');
    grid on;
    
    % 角速率
    subplot(3,2,6);
    plot(t, rad2deg(x(:,10)), t, rad2deg(x(:,11)), t, rad2deg(x(:,12)));
    ylabel('角速率 (deg/s)');
    xlabel('时间 (s)');
    title('角速率');
    legend('p', 'q', 'r');
    grid on;
end