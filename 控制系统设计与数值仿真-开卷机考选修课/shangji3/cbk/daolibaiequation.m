function statedot = daolibaiequation(t,state, M, u)
  
global m1
global m2    
global    l 
global    g     
    x = state(1);        % 小车位置
    dx = state(2);       % 小车速度
    theta = state(3);    % 摆杆角度
    dtheta = state(4);   % 摆杆角速度
    
    % 计算公共分母项，避免重复计算
    denominator = m1 + m2 * sin(theta)^2;
    
    % 计算 x 的二阶导数 (小车加速度)
    ddx = (m2 * g * cos(theta) * sin(theta) - m2 * l * dtheta^2 * sin(theta)) / denominator ...
          + (1 / denominator) * u ...
          + (cos(theta) / (denominator * l)) * M;
    
    % 计算 theta 的二阶导数 (摆杆角加速度)
    ddtheta = ((m1 + m2) * g * sin(theta) - m2 * l * dtheta^2 * cos(theta) * sin(theta)) / (denominator * l) ...
              + (cos(theta) / (denominator * l)) * u ...
              + ((m1 + m2) / (denominator * m2 * l^2)) * M;
    
    % 返回状态导数向量
    statedot = [dx; ddx; dtheta; ddtheta];
end