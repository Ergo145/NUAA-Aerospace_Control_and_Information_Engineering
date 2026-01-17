function [r_vec, v_vec] = get_planet_state(planet_id, date_input)
% GET_PLANET_STATE 计算行星在指定日期的日心位置和速度矢量
% 输入:
%   planet_id: 2=金星, 3=地球, 4=火星
%   date_input: datetime对象 或 儒略日数值
% 输出:
%   r_vec: 位置矢量 (km)
%   v_vec: 速度矢量 (km/s)

    % 太阳引力常数 (km^3/s^2)
    mu = 1.32712440018e11; 
    AU = 149597870.7; 

    % --- 日期处理逻辑 (修复报错) ---
    if isdatetime(date_input)
        jd = juliandate(date_input);
    elseif isnumeric(date_input)
        jd = date_input;
    else
        % 备用：强制指定英文格式解析字符串
        t = datetime(date_input, 'InputFormat', 'dd-MMM-yyyy', 'Locale', 'en_US');
        jd = juliandate(t);
    end
    
    % 计算儒略世纪数 T0 (相对于 J2000)
    T0 = (jd - 2451545) / 36525;

    % --- 表 3.12 轨道根数数据 ---
    % 数据结构: [Base_Value, Rate]
    % Rate 单位转换: arcsec/Cy -> deg/Cy (除以 3600)
    
    % 2: Venus (金星)
    p_data(2).a = [0.72333199, 0.00000092];
    p_data(2).e = [0.00677323, -0.00004938];
    p_data(2).i = [3.39471, -2.86/3600];
    p_data(2).Om = [76.68069, -996.89/3600];
    p_data(2).vp = [131.53298, -108.8/3600];
    p_data(2).L  = [181.97973, 210664136.1/3600];
    
    % 3: Earth (地球)
    p_data(3).a = [1.00000011, -0.00000005];
    p_data(3).e = [0.01671022, -0.00003804];
    p_data(3).i = [0.00005, -46.94/3600];
    p_data(3).Om = [-11.26064, -18228.25/3600];
    p_data(3).vp = [102.94719, 1198.28/3600];
    p_data(3).L  = [100.46435, 129597740.6/3600];

    % 4: Mars (火星)
    p_data(4).a = [1.52366231, -0.00007221];
    p_data(4).e = [0.09341233, 0.00011902];
    p_data(4).i = [1.85061, -25.47/3600];
    p_data(4).Om = [49.57854, -1020.19/3600];
    p_data(4).vp = [336.04684, 1560.78/3600]; % 已校对 OCR 识别
    p_data(4).L  = [355.45332, 68905103.78/3600];

    % 计算当前时刻根数
    curr = p_data(planet_id);
    a = curr.a(1) + curr.a(2) * T0;
    e = curr.e(1) + curr.e(2) * T0;
    i = deg2rad(curr.i(1) + curr.i(2) * T0);
    Om = deg2rad(curr.Om(1) + curr.Om(2) * T0);
    vp = deg2rad(curr.vp(1) + curr.vp(2) * T0);
    L = deg2rad(curr.L(1) + curr.L(2) * T0);

    % 导出经典根数
    w = vp - Om; % 近地点幅角
    M = L - vp;  % 平近点角
    M = mod(M, 2*pi);
    
    % 求解开普勒方程 M = E - e*sin(E)
    E = M; 
    iter = 0;
    while(1)
        E_new = E - (E - e*sin(E) - M) / (1 - e*cos(E));
        if abs(E_new - E) < 1e-8, break; end
        E = E_new;
        iter = iter + 1;
        if iter > 100, break; end
    end
    
    % 计算真近点角 nu 和 半通径 p
    nu = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2));
    p = a * AU * (1 - e^2);

    % PQW 坐标系状态
    r_pqw = (p / (1 + e*cos(nu))) * [cos(nu); sin(nu); 0];
    v_pqw = sqrt(mu/p) * [-sin(nu); (e + cos(nu)); 0];

    % 旋转至日心黄道坐标系 (ECI)
    R3_W = [cos(-Om) sin(-Om) 0; -sin(-Om) cos(-Om) 0; 0 0 1];
    R1_i = [1 0 0; 0 cos(-i) sin(-i); 0 -sin(-i) cos(-i)];
    R3_w = [cos(-w) sin(-w) 0; -sin(-w) cos(-w) 0; 0 0 1];
    
    Q_px = (R3_w * R1_i * R3_W)'; 
    r_vec = (Q_px * r_pqw)';
    v_vec = (Q_px * v_pqw)';
end