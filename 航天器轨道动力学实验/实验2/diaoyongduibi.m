clc;clear;close all;
%% 初始参数设定
a = 10000;      % 半长轴 (km)
e = 0.3;        % 偏心率 (0 < e < 1)
i_deg = 45;     % 轨道倾角 (degrees)
Omega_deg = 60; % 升交点赤经 (degrees)
omega_deg = 30; % 近地点幅角 (degrees)
M0_deg = 45;    % 卫星当前的初始平近点角

% 物理常数
mu = 398600;    % 地球引力常数
Re = 6378;      % 地球半径 (km)

%% 对比结果
E0_rad = M_to_E(M0_deg, e);
theta_deg = E_to_theta(E0_rad, e);
coe1(1)=a;
coe1(2)=e;
coe1(3)=i_deg;
coe1(4)=Omega_deg;
coe1(5)=omega_deg;
coe1(6)=theta_deg;
% 调用sv_from_coe
[r1,v1]=sv_from_coe(coe1);
[r2,v2]=sv_from_coe(coe1);
% 调用coe_from_sv
coe2 = coe_from_sv(r1, v1);
% 结果对比
compare1 = [coe1;coe2];
compare2 = [r1,r2];
compare3 = [v1,v2];

function theta = E_to_theta(E, e)
    % 将偏近点角 E 转换为真近点角 theta
    theta = 2 * atan(sqrt((1 + e) / (1 - e)) * tan(E / 2));
    theta = rad2deg(theta);
end

function E = M_to_E(M, e)
    % 将平近点角 M 转换为偏近点角 E (牛顿法求解)
    M = deg2rad(M);
    % 初始猜测 (适用于所有 e)
    if M < pi
        E = M + e;
    else
        E = M - e;
    end
    
    % 牛顿法迭代
    tolerance = 1e-8;
    for k = 1:20 
        E_new = E - (E - e * sin(E) - M) / (1 - e * cos(E));
        if abs(E_new - E) < tolerance
            break;
        end
        E = E_new;
    end
end

