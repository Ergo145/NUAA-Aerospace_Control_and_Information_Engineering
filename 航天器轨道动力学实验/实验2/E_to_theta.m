function theta = E_to_theta(E, e)
    % 将偏近点角 E 转换为真近点角 theta
    theta = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2));
    theta = rad2deg(theta);
end