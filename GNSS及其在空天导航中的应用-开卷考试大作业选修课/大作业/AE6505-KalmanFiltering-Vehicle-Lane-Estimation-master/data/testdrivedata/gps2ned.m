function dx = gps2ned(y,y0)
% gps2ned.m  
% GPS2NED 将GPS坐标转换为NED坐标系中的位移
% 输入:
%   y - 当前GPS坐标 [纬度, 经度, 高度]
%   y0 - 参考GPS坐标 [纬度, 经度, 高度]
% 输出:
%   dx - NED坐标系中的位移 [北, 东, 下]
    lat = deg2rad(y(1));
    long = deg2rad(y(2));
    alt = deg2rad(y(3));

    dy = deg2rad(y - y0);
    dlat = dy(1);
    dlong = dy(2);
    dalt = dy(3);
    
    R_e = 6378.137;     % earth radius @ equator
    R_p = 6356.752;     % earth radius @ pole
    
    R = sqrt(...
        (((R_e^2)*cos(lat))^2 + ((R_p^2)*sin(lat))^2) /...
        (((R_e)*cos(lat))^2 + ((R_p)*sin(lat))^2)...
        ) * 1000 + alt;
         % earth radius @ lat [m]
    
    dN = dlat * R;
    dE = dlong * R;
    dD = -dalt * R;
    dx = [dN; dE; dD];
end