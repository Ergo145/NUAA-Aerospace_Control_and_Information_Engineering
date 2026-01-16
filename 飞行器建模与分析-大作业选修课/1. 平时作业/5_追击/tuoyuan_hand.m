clc;clear;close;

%% 参数设定
w = 10 ;    %导弹速度
t = 100 ;   %仿真时间
step = 1 / 1000 ;
%图窗设定
figure('Position', [100, 100, 1200, 800]);

%% 仿真
t_tuo = linspace(0,2*pi,1000);
X = cos(t_tuo) * 20 + 10;
Y = sin(t_tuo) * 50 + 20;

%%仿真开始
for i=0:step:t
    %导弹运动
    
end