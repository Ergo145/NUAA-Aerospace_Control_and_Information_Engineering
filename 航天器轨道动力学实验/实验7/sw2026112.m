% 清除工作区
clear; clc; close all;

% 1. 定义传递函数变量
s = tf('s');

% 2. 定义推导出的等效开环传递函数
% 分子是 (s + 4)
% 分母是 (s^2 + 4s + 5)
Sys_eq = (s + 4) / (s^2 + 4*s + 5);

% 3. 绘制根轨迹
figure;
rlocus(Sys_eq);

% 添加标题和网格
title('关于参数 a 的闭环系统根轨迹 (等效传递函数)');
grid on;

% --- 额外功能：验证图片中的第二问 ---
% 图片第二问要求：当系统响应包含 e^(-4t)*sin(wt) 时求 a。
% 这意味着闭环极点的实部必须是 -4。
% 我们可以在图上通过点击数据游标验证，或者用计算方式验证：
% 计算 a=4 时的闭环极点
a_val = 4;
ClosedLoop_Char_Eq_Roots = roots([1, (4+a_val), (4*a_val+5)]);
disp(['当 a=4 时，闭环极点为：']);
disp(ClosedLoop_Char_Eq_Roots);