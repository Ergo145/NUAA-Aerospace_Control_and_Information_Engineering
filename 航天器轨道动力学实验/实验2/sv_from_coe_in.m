function [r, v] = sv_from_coe_in(coe)

    mu = 398600.4418; % 地球引力常数 km^3/s^2
    h = coe(1);
    e = coe(2);
    RA = coe(3);
    incl = coe(4);
    w = coe(5);
    TA = coe(6);

    % 计算轨道位置和速度向量（在轨道平面坐标系中）
    rp = (h^2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0] + sin(TA)*[0;1;0]);
    vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);

    % 定义旋转变换矩阵（从轨道平面坐标系转换到惯性坐标系）
    % 绕 Z 轴旋转 RA（升交点赤经）
    R3_W = [ cos(RA) sin(RA) 0
        -sin(RA) cos(RA) 0
        0       0       1];
    % 绕 X 轴旋转 incl（轨道倾角）
    RL_i = [ 1     0          0
        0 cos(incl) sin(incl)
        0 -sin(incl) cos(incl)];
    % 绕 Z 轴旋转 w（近地点幅角）
    R3_w = [ cos(w) sin(w) 0
        -sin(w) cos(w) 0
        0      0      1];

    % 组合旋转矩阵（从轨道平面到惯性坐标系）
    Q_pX = R3_W' * RL_i' * R3_w';

    % 将位置和速度向量转换到惯性坐标系
    r = Q_pX * rp;
    v = Q_pX * vp;

    % 将列向量转置为行向量
    r = r';
    v = v';
end