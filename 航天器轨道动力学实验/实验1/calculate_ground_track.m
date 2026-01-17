 function [lon, lat] = calculate_ground_track(T_total)
        % T_total: 仿真总时长 (秒)
        
        t_span = 0 : 30 : T_total; % 每30秒一个点
        lat_calc = []; lon_calc = [];
        n = sqrt(mu/a^3); % 平均角速度
        
        for t = t_span
            
            % 求解开普勒方程求真近点角 nu
            M = M0 + n * t;   % 当前平近点角
            
            % 迭代法解 E (偏近点角)
            E = M; 
            for iter = 1:5
                E = M + e * sin(E);
            end
            
            % 计算真近点角 nu
            sin_nu = (sqrt(1-e^2)*sin(E)) / (1-e*cos(E));
            cos_nu = (cos(E)-e) / (1-e*cos(E));
            nu = atan2(sin_nu, cos_nu);
            
            % 更新 coe 并获取 ECI 位置
            current_coe = coe;
            current_coe(6) = rad2deg(nu);
            [r_eci, ~] = sv_from_coe(current_coe);
            
            % 转换到 ECEF (地固系)
            theta_g = we * t; % 地球自转角度
            
            R_z = [cos(theta_g) sin(theta_g) 0;
                  -sin(theta_g) cos(theta_g) 0;
                   0            0           1];
                   
            r_ecef = R_z * r_eci;
            
            % 计算经纬度
            r_mag = norm(r_ecef);
            phi = asin(r_ecef(3) / r_mag);     % 纬度
            lambda = atan2(r_ecef(2), r_ecef(1)); % 经度
            
            lat_calc = [lat_calc, rad2deg(phi)];
            lon_calc = [lon_calc, rad2deg(lambda)];
        end
        
        % 处理经度跳变 (美化绘图)
        for k = 2:length(lon_calc)
            if abs(lon_calc(k) - lon_calc(k-1)) > 100
                lon_calc(k-1) = NaN; % 断开线条
            end
        end
        lon = lon_calc;
        lat = lat_calc;
    end