function coe = coe_from_sv(r, v)
% 输入 r: 位置矢量 (km), v: 速度矢量 (km/s)
% 输出 coe: [a (km), e, i (deg), Omega (deg), omega (deg), theta (deg)]

mu = 398600.4418; % 地球引力常数 km^3/s^2

r_mag = norm(r);
v_mag = norm(v);

h = cross(r, v);
h_mag = norm(h);

i = acos(h(3) / h_mag);

n = cross([0; 0; 1], h);
n_mag = norm(n);

if n_mag ~= 0
    Omega = acos(n(1) / n_mag);
    if n(2) < 0
        Omega = 2 * pi - Omega;
    end
else
    Omega = 0;
end

e_vec = (cross(v, h) / mu) - (r / r_mag);
e = norm(e_vec);

if abs(e - 1) < 1e-6
    a = Inf;
else
    a = 1 / ( (2/r_mag) - (v_mag^2/mu) );
end

if n_mag ~= 0
    omega = acos(dot(n, e_vec) / (n_mag * e));
    if dot(cross(n, e_vec), h) < 0
        omega = 2 * pi - omega;
    end
else
    omega = acos(e_vec(1) / e);
    if e_vec(2) < 0
        omega = 2 * pi - omega;
    end
end

theta = acos(dot(e_vec, r) / (e * r_mag));
if dot(r, v) < 0
    theta = 2 * pi - theta;
end

coe = [a, e, rad2deg(i), 180-rad2deg(Omega), 180-rad2deg(omega), rad2deg(theta)];

end