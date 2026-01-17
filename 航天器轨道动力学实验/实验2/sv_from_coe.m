function [r, v] = sv_from_coe(coe)
% 输入 coe: [a (km), e, i (deg), Omega (deg), omega (deg), theta (deg)]
% 输出 r: 位置矢量 (km), v: 速度矢量 (km/s)

mu = 398600.4418; % 地球引力常数 km^3/s^2

a = coe(1); e = coe(2); i = deg2rad(coe(3));
Omega = deg2rad(coe(4)); w = deg2rad(coe(5)); theta = deg2rad(coe(6));

p = a * (1 - e^2);
r_pqw = (p / (1 + e * cos(theta))) * [cos(theta); sin(theta); 0];
v_pqw = sqrt(mu/p) * [-sin(theta); e + cos(theta); 0];

R3_W = [cos(-Omega) sin(-Omega) 0; -sin(-Omega) cos(-Omega) 0; 0 0 1];
R1_i = [1 0 0; 0 cos(-i) sin(-i); 0 -sin(-i) cos(-i)];
R3_w = [cos(-w) sin(-w) 0; -sin(-w) cos(-w) 0; 0 0 1];

Q_p2i = (R3_w * R1_i * R3_W)'; 

r = Q_p2i * r_pqw;
v = Q_p2i * v_pqw;
end
