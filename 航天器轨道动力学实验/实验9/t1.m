%% 实验内容1：绘制等效势能曲面 (Equivalent Potential Energy Surface)
clc; clear; close all;

% --- 参数设置 ---
mu = 0.3; % 质量比 (参考文中图12.2的设置) 

% --- 网格生成 ---
[x, y] = meshgrid(-2:0.05:3, -2:0.05:2);

% --- 计算距离 r1, r2 ---
% P1 位置 (-mu, 0), P2 位置 (1-mu, 0)
r1 = sqrt((x + mu).^2 + y.^2);
r2 = sqrt((x - 1 + mu).^2 + y.^2);

% --- 计算等效势能 Omega (公式 12.7) [cite: 97] ---
% 第一项：离心势能; 第二、三项：引力势能
Omega = 0.5 * (x.^2 + y.^2) + (1 - mu) ./ r1 + mu ./ r2;

% --- 处理奇点 (为了绘图美观，截断无穷大的峰值) ---
Omega(Omega > 3.5) = 3.5; 

% --- 绘图 ---
figure('Color', 'w');
surf(x, y, Omega, 'EdgeColor', 'none');
hold on;
contour3(x, y, Omega, 20, 'k'); % 叠加等高线
colormap(gray); % 使用文中类似的灰度图
colorbar;

% --- 标注 ---
title(['等效势能曲面 (\mu = ', num2str(mu), ')']);
xlabel('x'); ylabel('y'); zlabel('\Omega(x,y)');
view(-30, 50); % 调整视角以匹配图 12.2
axis tight;
grid on;