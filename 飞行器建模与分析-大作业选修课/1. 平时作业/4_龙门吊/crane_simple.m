function dy = crane_simple(t,y,m0,m,l,g,F_func)
    % 状态变量
    x = y(1); dx = y(2);
    theta = y(3); dtheta = y(4);
    F = F_func(t);

    % 简化方程
    ddx = - (g*m/m0)*theta + F/m0;
    ddtheta = - (g*(m0+m)/(m0*l))*theta + F/(m0*l);

    % 返回导数
    dy = [dx; ddx; dtheta; ddtheta];
end
