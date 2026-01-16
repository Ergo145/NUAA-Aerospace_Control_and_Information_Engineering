% 实验一：绘制 G(s)=50/(s(0.1s+1)) 的闭环系统阶跃响应
clc; clear; close all;

% 设置参数
K = 50;  % 开环增益
t = 0.1; % 时间常数

T_raw = readmatrix('data.xlsx');% 读取数据
w = T_raw(1,3:end);           % 对应原表第 1 行 (频率 w)

% 构造开环传递函数 G(s) = K/(s*(t*s+1))
num = K;
den = conv([1, 0], [t, 1]); % [s] * [0.1s+1] = [s(0.1s+1)]
g0 = tf(num, den)

% 求单位反馈闭环传递函数
g = feedback(g0, 1)

% 计算频率特性
size = length(w);
for i=1:size
    [A_theo(i),phi_theo(i)] = bode(g,w(i));
end
wb = bandwidth(g)
% 绘制阶跃响应曲线
figure;
step(g, 'b-');
title("G(s)=50/(s(0.1s+1)) 的闭环系统阶跃响应");
xlabel("时间 t");
ylabel("振幅");
grid on;
figure;
bode(g, 'b-');
title("G(s)=50/(s(0.1s+1)) 的闭环系统阶跃响应");
xlabel("时间 t");
ylabel("振幅");
grid on;