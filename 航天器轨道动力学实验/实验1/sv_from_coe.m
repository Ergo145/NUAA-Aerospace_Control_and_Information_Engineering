function [r, v] = sv_from_coe(coe)

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
% % 输入 coe: [a (km), e, i (deg), Omega (deg), omega (deg), nu (deg)]
% % 输出 r: 位置矢量 (km), v: 速度矢量 (km/s)
%
% mu = 398600.4418; % 地球引力常数 km^3/s^2
%
% a = coe(1); e = coe(2); i = deg2rad(coe(3));
% Omega = deg2rad(coe(4)); w = deg2rad(coe(5)); nu = deg2rad(coe(6));
%
% % 1. 计算轨道平面(PQW)中的位置和速度
% p = a * (1 - e^2);
% r_pqw = (p / (1 + e * cos(nu))) * [cos(nu); sin(nu); 0];
% v_pqw = sqrt(mu/p) * [-sin(nu); e + cos(nu); 0];
%
% % 2. 旋转矩阵 (3-1-3 欧拉角序列: Omega -> i -> w)
% % 由于是从轨道系转到地心惯性系，顺序是 Rz(-Omega)*Rx(-i)*Rz(-w) 的逆变换
% % 即 R3(-w) -> R1(-i) -> R3(-Omega) 的转置... 或者直接写旋转矩阵
%
% R3_W = [cos(-Omega) sin(-Omega) 0; -sin(-Omega) cos(-Omega) 0; 0 0 1];
% R1_i = [1 0 0; 0 cos(-i) sin(-i); 0 -sin(-i) cos(-i)];
% R3_w = [cos(-w) sin(-w) 0; -sin(-w) cos(-w) 0; 0 0 1];
%
% Q_p2i = (R3_w * R1_i * R3_W)'; % 转置即为从 PQW 到 ECI 的旋转
%
% % 3. 转换到 ECI 坐标系
% r = Q_p2i * r_pqw;
% v = Q_p2i * v_pqw;