%% 实验内容2：零速度面与运动禁止区域 (Forbidden Regions)
clc; clear; close all;

% --- 参数设置 ---
mu = 0.3; % 质量比
C_values = [4.0, 3.6, 3.2, 2.8]; % 对应图中 (a)(b)(c)(d) 四种情况的典型C值

% --- 网格生成 ---
[x, y] = meshgrid(-2:0.01:2, -2:0.01:2);
r1 = sqrt((x + mu).^2 + y.^2);
r2 = sqrt((x - 1 + mu).^2 + y.^2);
Omega = 0.5 * (x.^2 + y.^2) + (1 - mu) ./ r1 + mu ./ r2;

% --- 绘图循环 ---
figure('Color', 'w', 'Position', [100, 100, 800, 700]);

titles = {'(a) C > C1 (封闭)', '(b) C2 < C < C1 (L1连通)', ...
          '(c) C3 < C < C2 (L2连通)', '(d) C < C3 (L3连通)'};

for i = 1:4
    subplot(2, 2, i);
    C = C_values(i);
    
    % 绘制等高线 2*Omega = C [cite: 17]
    % 只有 2*Omega >= C 的区域是可运动区域 (v^2 >= 0) 
    % 这里的contour绘制的是边界
    contourf(x, y, 2*Omega, [C, C], 'LineWidth', 1.5, 'LineColor', 'k'); 
    
    % 填充禁止区域 (灰色)
    hold on;
    % 利用不同颜色区分可行域和禁区，这里简单勾勒边界
    caxis([C-0.1, C+0.1]); 
    
    % 标记主天体位置
    plot(-mu, 0, 'k.', 'MarkerSize', 15); text(-mu, -0.2, 'P1');
    plot(1-mu, 0, 'k.', 'MarkerSize', 15); text(1-mu, -0.2, 'P2');
    
    title(['C = ', num2str(C)]);
    xlabel('x'); ylabel('y');
    axis equal; grid on;
    xlim([-1.5, 1.5]); ylim([-1.5, 1.5]);
end
sgtitle('零速度面的几何结构随雅可比常数 C 值的变化');