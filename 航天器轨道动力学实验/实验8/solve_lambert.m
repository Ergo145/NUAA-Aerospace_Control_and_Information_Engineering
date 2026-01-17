function [v1, v2] = solve_lambert(r1, r2, dt)
% SOLVE_LAMBERT 简化版通用变量法兰伯特求解器
% 输入: r1, r2 (位置矢量 km), dt (飞行时间 sec)
% 输出: v1 (出发速度), v2 (到达速度)

    mu = 1.32712440018e11;
    r1_mag = norm(r1);
    r2_mag = norm(r2);
    
    cross12 = cross(r1, r2);
    % 假设顺行轨道
    theta = acos(dot(r1, r2) / (r1_mag * r2_mag));
    if cross12(3) < 0
        theta = 2*pi - theta;
    end
    
    A = sin(theta) * sqrt(r1_mag * r2_mag / (1 - cos(theta)));
    
    % 迭代求解 z
    z = 0;
    iter = 0;
    while iter < 1000
        [S, C] = stumpff(z);
        y = r1_mag + r2_mag + A * (z*S - 1) / sqrt(C);
        
        if y < 0, y = 1e-5; end % 防止数值错误
        
        chi = sqrt(y/C);
        dt_iter = (chi^3 * S + A * sqrt(y)) / sqrt(mu);
        
        if abs(dt_iter - dt) < 1e-5
            break;
        end
        
        % 简单的步长调整 (实际应用建议使用牛顿法)
        step = (dt - dt_iter) / 1e5; 
        % 限制步长防止发散
        if step > 1, step = 1; end
        if step < -1, step = -1; end
        
        z = z + step; 
        iter = iter + 1;
    end
    
    f = 1 - y/r1_mag;
    g = A * sqrt(y/mu);
    g_dot = 1 - y/r2_mag;
    
    v1 = (r2 - f*r1) / g;
    v2 = (g_dot*r2 - r1) / g;
end

function [S, C] = stumpff(z)
    if z > 0
        s_z = sqrt(z);
        S = (s_z - sin(s_z)) / s_z^3;
        C = (1 - cos(s_z)) / z;
    elseif z < 0
        s_z = sqrt(-z);
        S = (sinh(s_z) - s_z) / s_z^3;
        C = (cosh(s_z) - 1) / (-z);
    else
        S = 1/6;
        C = 1/2;
    end
end