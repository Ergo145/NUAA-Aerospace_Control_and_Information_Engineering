clear; clc; format long g;

% --- 常量与观测数据 ---
mu = 398600.4418; Re = 6378.137; f_earth = 1/298.257; deg2rad = pi/180;
lat = 60.0 * deg2rad; alt = 0.5; % km

% 数据表: [t(min), lst(deg), alpha(deg), delta(deg)]
data = [0.0, 150.000, 157.783, 24.2403;
        5.0, 151.253, 159.221, 27.2993;
        10.0, 152.507, 160.526, 29.8982];

% 1. 预处理
t_s = data(:,1) * 60;
tau1 = t_s(1) - t_s(2); % -300s
tau3 = t_s(3) - t_s(2); % +300s
tau = tau3 - tau1;      % 600s 

R = zeros(3,3); L = zeros(3,3);
for i = 1:3
    R(:,i) = iod_tools.calc_site_position(lat, alt, data(i,2)*deg2rad, Re, f_earth);
    ra = data(i,3)*deg2rad; dec = data(i,4)*deg2rad;
    L(:,i) = [cos(dec)*cos(ra); cos(dec)*sin(ra); sin(dec)];
end

% 2. 计算 D0, Dij
D0 = dot(L(:,1), cross(L(:,2), L(:,3)));
D = zeros(3,4); % D(j, k) -> 公式中的 D_jk
L_cross = {cross(L(:,2),L(:,3)), cross(L(:,1),L(:,3)), cross(L(:,1),L(:,2))};
for j=1:3 % 对应公式下标 1, 2, 3
    for k=1:3 % 对应观测点 R1, R2, R3
        D(k,j) = dot(R(:,k), L_cross{j});
    end
end
% 注意：代码索引 D(k,j) 对应 R_k 点乘 L_cross_j -> 公式 D_kj

% 3. 计算 A, B, E 
A = (1/D0) * (-D(1,2)*(tau3/tau) + D(2,2) + D(3,2)*(tau1/tau));
B = (1/(6*D0)) * (D(1,2)*(tau3^2-tau^2)*(tau3/tau) + D(3,2)*(tau^2-tau1^2)*(tau1/tau));
E = dot(R(:,2), L(:,2));

% 4. 求解八次方程 x^8 + a*x^6 + b*x^3 + c = 0
coeffs = [1, 0, -(A^2+2*A*E+norm(R(:,2))^2), 0, 0, -2*mu*B*(A+E), 0, 0, -mu^2*B^2];
roots_r = roots(coeffs);
r2_val = 0;
for r = roots_r'
    if isreal(r) && r > Re, r2_val = real(r); break; end
end

% 5. 计算斜距 rho [cite: 25, 156-166]
u = mu / r2_val^3;
c1 = (tau3/tau) * (1 + u/6*(tau^2-tau3^2)); %
c3 = -(tau1/tau) * (1 + u/6*(tau^2-tau1^2)); %
% 使用线性方程组公式求解准确的 rho
rho1 = (1/D0)*(-D(1,1) + (1/c1)*D(2,1) - (c3/c1)*D(3,1));
rho2 = (1/D0)*(-c1*D(1,2) + D(2,2) - c3*D(3,2));
rho3 = (1/D0)*(-(c1/c3)*D(1,3) + (1/c3)*D(2,3) - D(3,3));

% 6. 计算状态矢量
r_vec = zeros(3,3);
r_vec(:,1) = R(:,1) + rho1*L(:,1);
r_vec(:,2) = R(:,2) + rho2*L(:,2);
r_vec(:,3) = R(:,3) + rho3*L(:,3);

% 速度矢量 (级数近似) 
f1 = 1 - 0.5*u*tau1^2; f3 = 1 - 0.5*u*tau3^2;
g1 = tau1 - (1/6)*u*tau1^3; g3 = tau3 - (1/6)*u*tau3^3;
v2_vec = (1/(f1*g3 - f3*g1)) * (-f3*r_vec(:,1) + f1*r_vec(:,3));

fprintf('--- 高斯法初步结果 ---\n');
fprintf('r2: [%10.4f, %10.4f, %10.4f] km\n', r_vec(:,2));
fprintf('v2: [%10.4f, %10.4f, %10.4f] km/s\n', v2_vec);

% 保存结果供后续任务使用
save('task1_result.mat');
coe = coe_from_sv(r_vec(:,2), v2_vec);