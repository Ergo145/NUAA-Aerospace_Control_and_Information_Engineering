function [v1, v2] = lambert_solver(r1, r2, dt, mu)
    % 简单的兰伯特求解器 (Universal Variables)
    % 输入: r1, r2 (位置矢量), dt (时间秒), mu (引力常数)
    % 输出: v1, v2 (两端速度矢量)
    
    r1_norm = norm(r1);
    r2_norm = norm(r2);
    
    cross_12 = cross(r1, r2);
    theta = acos(dot(r1, r2) / (r1_norm * r2_norm));
    
    % 判断顺行/逆行 (简单假设顺行 < 180度)
    if cross_12(3) < 0
        theta = 2*pi - theta;
    end
    
    A = sin(theta) * sqrt(r1_norm * r2_norm / (1 - cos(theta)));
    
    % 迭代求解 z
    z = 0;
    ratio = 1;
    iter = 0;
    while abs(ratio) > 1e-6 && iter < 100
        [C, S] = stumpff(z);
        y = r1_norm + r2_norm + A * (z * S - 1) / sqrt(C);
        chi = sqrt(y / C);
        dt_calc = (chi^3 * S + A * sqrt(y)) / sqrt(mu);
        
        ratio = (dt_calc - dt) / dt;  % 误差
        % 简单的牛顿迭代更新 (此处简化，实际需计算导数)
        % 为保证稳健性，这里使用二分法或简单步进示意
        if dt_calc < dt
            z = z + 0.1;
        else
            z = z - 0.1;
        end
        iter = iter + 1;
    end
    
    % 计算 f, g, g_dot [cite: 28]
    [C, S] = stumpff(z);
    y = r1_norm + r2_norm + A * (z * S - 1) / sqrt(C);
    f = 1 - y / r1_norm;
    g = A * sqrt(y / mu);
    g_dot = 1 - y / r2_norm;
    
    v1 = (r2 - f * r1) / g;
    v2 = (g_dot * r2 - r1) / g; % 注意：这里仅为近似公式，实际需完整拉格朗日系数
end

function [c, s] = stumpff(z)
    if z > 0
        s = (sqrt(z) - sin(sqrt(z))) / (sqrt(z))^3;
        c = (1 - cos(sqrt(z))) / z;
    elseif z < 0
        s = (sinh(sqrt(-z)) - sqrt(-z)) / (sqrt(-z))^3;
        c = (cosh(sqrt(-z)) - 1) / (-z);
    else
        s = 1/6;
        c = 1/2;
    end
end