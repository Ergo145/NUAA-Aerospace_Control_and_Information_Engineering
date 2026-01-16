function [dangle] = euler_kinematics(gyro,euler)
% euler_kinematics.m
% EULER_KINEMATICS Summary of this function goes here
% Detailed explanation goes here
% EULER_KINEMATICS 计算欧拉角导数
% 输入:
%   gyro - 陀螺仪角速度 (rad/s)
%   euler - 当前欧拉角 [滚转, 俯仰, 偏航] (rad)
% 输出:
%   dangle - 欧拉角导数
phi = euler(1);
theta = euler(2);
psi = euler(3);

R = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi), -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

dangle = R * gyro;
end

