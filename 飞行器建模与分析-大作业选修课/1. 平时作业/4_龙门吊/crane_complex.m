function dy = crane_complex(t,y,m0,m,l,g,F_func)
    % 状态变量
    x = y(1); dx = y(2);
    theta = y(3); dtheta = y(4);
    F = F_func(t);

    % 动态方程
    ddx = (F - g*m*sin(theta)*cos(theta) - m*l*dtheta^2*sin(theta)) / ...
          (m0 + m*sin(theta)^2);

    ddtheta = (F*cos(theta) - m*l*dtheta^2*sin(theta)*cos(theta) ...
              - (m0+m)*g*sin(theta)) / ...
              (l*(m0 + m*sin(theta)^2));

    % 返回导数
    dy = [dx; ddx; dtheta; ddtheta];
end
