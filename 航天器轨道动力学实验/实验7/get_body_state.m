function [r, v] = get_body_state(body_name, days, mu)
    % 简化的二体模型星历生成器
    % 输入: body_name, days (距离J2000的天数)
    
    % 轨道根数 [a(km), e, i(rad), Omega(rad), w(rad), M0(rad)]
    if strcmp(body_name, 'Earth')
        coe = [1.496e8, 0.0167, 0, 0, 0, 0]; 
        % 这里的M需要根据 days 更新
        n = sqrt(mu / coe(1)^3);
        M = coe(6) + n * (days * 86400);
    else 
        % 模拟 "2013 WA44" 或类似近地小行星
        % 假设一个比地球稍大的椭圆轨道
        coe = [1.0975044 * 1.496e8, 0.0579408, deg2rad(2.29904), deg2rad(55.98285), deg2rad(177.65979), deg2rad(338.54721)];
        n = sqrt(mu / coe(1)^3);
        M = coe(6) + n * (days * 86400);
    end
    
    % 求解开普勒方程 M = E - e*sin(E)
    E = M; 
    for k = 1:10
        E = M + coe(2)*sin(E);
    end
    
    % 计算 perifocal 坐标系状态
    a = coe(1); e = coe(2);
    p = a * (1 - e^2);
    r_pqw = [p*cos(E-e*sin(E)); p*sqrt(1-e^2)*sin(E); 0]; % 简化版，实际需通过真近点角计算
    % 这里使用真近点角 nu 这种更通用的方式：
    nu = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2));
    r_mag = p / (1 + e*cos(nu));
    r_pqw = [r_mag*cos(nu); r_mag*sin(nu); 0];
    v_pqw = sqrt(mu/p) * [-sin(nu); e+cos(nu); 0];
    
    % 旋转矩阵 (3-1-3 序列)
    O = coe(4); w = coe(5); i = coe(3);
    R3_O = [cos(O) sin(O) 0; -sin(O) cos(O) 0; 0 0 1];
    R1_i = [1 0 0; 0 cos(i) sin(i); 0 -sin(i) cos(i)];
    R3_w = [cos(w) sin(w) 0; -sin(w) cos(w) 0; 0 0 1];
    Q_px = (R3_w * R1_i * R3_O)'; % 转换矩阵
    
    r = (Q_px * r_pqw)';
    v = (Q_px * v_pqw)';
end