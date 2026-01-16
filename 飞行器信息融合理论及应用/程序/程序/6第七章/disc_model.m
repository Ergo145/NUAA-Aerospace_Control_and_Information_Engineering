function [Phi, Qd] = disc_model(F, G, Q, T)
% 功能描述：计算连续系统方程离散化模型参数，适用于定常系统
% 输出参数：Phi，离散系统矩阵
%          Qd, 离散系统噪声矩阵
% 输入参数：F，连续系统矩阵
%          G, 连续噪声矩阵
%          Q，连续系统噪声功率谱密度阵
%          T，离散时间间隔
% 作    者：Jr9910
% 日    期：2014/12/15
% 版    本：V1.0 
n = size(F);
mat_a = [-F, G * Q * G'; zeros(3) F'] * T;
mat_b = expm(mat_a);
Phi = mat_b((n+1: 2*n), (n+1: 2*n))';
Qd = Phi * mat_b((1:n), (n+1: 2*n));
end