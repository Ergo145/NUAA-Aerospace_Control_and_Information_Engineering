function dy = ned2gps(dx, y)
% ned2gps.m
% NED2GPS 将NED位移转换为GPS坐标变化
% 输入:
%   dx - NED坐标系中的位移 [北, 东, 下]
%   y - 参考GPS坐标 [纬度, 经度, 高度]
% 输出:
%   dy - GPS坐标变化 [纬度变化, 经度变化, 高度变化]
    lat = deg2rad(y(1));
    long = deg2rad(y(2));
    alt = deg2rad(y(3));

    R_e = 6378.137;     % earth radius @ equator
    R_p = 6356.752;     % earth radius @ pole
    
    R = sqrt(...
        (((R_e^2)*cos(lat))^2 + ((R_p^2)*sin(lat))^2) /...
        (((R_e)*cos(lat))^2 + ((R_p)*sin(lat))^2)...
        ) * 1000 + alt;
         % earth radius @ lat [m]
    
    dlat = rad2deg(dx(1) / R);
    dlong = rad2deg(dx(2) / R);
    dalt = -dx(3);
    dy = [dlat; dlong; dalt];
end