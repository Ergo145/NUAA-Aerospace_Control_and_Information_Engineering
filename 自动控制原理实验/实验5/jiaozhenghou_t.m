% 清除环境
clc; clear; close all;

%% 1. 定义系统
s = tf('s');

% 原系统开环传递函数
G0 = 30 / (s * (0.1*s + 1) * (0.2*s + 1));

% 滞后校正装置
Gc = (3.65*s + 1) / (33.18*s + 1);

% 校正后开环传递函数
G_corr = Gc * G0;

%% 2. 构建闭环系统 (单位负反馈)
% 原系统闭环
Sys_Uncomp = feedback(G0, 1);

% 校正后闭环
Sys_Comp = feedback(G_corr, 1);

%% 3. 绘制阶跃响应对比图
figure(1);
% 设定时间范围，因为原系统不稳定，时间太长会导致幅值过大，影响观察
t = 0:0.01:15; 

% 绘图
figure(1);
step(Sys_Uncomp,t);
xlabel('时间 (Time) / s');
figure(2);
step(Sys_Comp,t);
grid on;
ylabel('幅值 (Amplitude)');