clear;
clc;
close all;
%% 初始化
line_width = 10;
line_count = 5;   %一系列平行线均平行于X轴
needle_length=rand()*10;
t = 100000;
%% 仿真
intersect_count=0;%相交次数

for i=1:t
    needle_center_y=random("normal",0,line_width);
    needle_theta=rand()*pi;
    needle_y = [needle_center_y-(needle_length*sin(needle_theta))/2,needle_center_y+(needle_length*sin(needle_theta))/2];
    for j=-2:2
        line_y=line_width*j;
        if needle_y(1)<=line_y && needle_y(2)>=line_y
            intersect_count = intersect_count + 1;
        end
    end
end

%% 计算估计的圆周率 
if intersect_count > 0
    estimated_pi = (2 * needle_length * t) / (line_width * intersect_count);
else
    estimated_pi = NaN;
end

fprintf('总投掷次数：%d\n', t);
fprintf('相交次数：%d\n', intersect_count);
fprintf('相交概率 P = 相交次数 / 总投掷次数 = %.4f\n', intersect_count / t);
fprintf('理论相交概率 P_theory = 2L / (pi*D) = %.4f\n', (2 * needle_length) / (pi * line_width));
fprintf('估计的圆周率 (2 * L * num_trials) / (D * intersect_count) = %.4f\n', estimated_pi);
fprintf('真实的圆周率 pi = %.4f\n', pi);