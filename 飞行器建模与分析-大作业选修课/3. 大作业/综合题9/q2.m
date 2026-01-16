clc;clear
% 情况2
for w2 = 0:0.01:10
    simout2=sim("qx2.slx");
    dis2=simout2.dis.data;
    if(min(dis2)<1e-5)
        break;
    end
end
disp(['模型2导弹最小速度',num2str(w2)]);
% 参数设置
t = simout2.dis.Time;
X2 = simout2.X.Data;
Y2 = simout2.Y.Data;
x2 = simout2.x.Data;
y2 = simout2.y.Data;

% 绘制椭圆轨迹
figure;
plot(X2, Y2, 'b-', 'LineWidth', 2, 'Color', [0.2 0.4 0.8]);
hold on;
grid on;

% 标记重要点
plot(10, 20, 'ks', 'MarkerSize', 10, 'MarkerFaceColor', [0.5 0.5 0.5]);  % 椭圆中心
plot(0, 0, '^', 'MarkerSize', 12, 'MarkerFaceColor', [0.8 0.2 0.2], ...
     'Color', [0.8 0.2 0.2]);  % 导弹起始点

% 绘制导弹轨迹
plot(x2, y2, 'r-', 'LineWidth', 2.5, 'Color', [0.9 0.1 0.1]);

% 标记导弹起点（如果与上面的点重复，可以省略）
plot(0, 0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', [0.9 0.1 0.1], ...
     'Color', [0.9 0.1 0.1]);

% 标记击中点（假设击中时距离最小）
[~, hit_idx2] = min(simout2.dis.Data);
hit_x2 = x2(hit_idx2);
hit_y2 = y2(hit_idx2);
plot(hit_x2, hit_y2, 'pentagram', 'MarkerSize', 15, 'MarkerFaceColor', [0.9 0.7 0.1], ...
     'Color', [0.9 0.5 0], 'LineWidth', 2);

% 坐标轴和标题
xlabel('X 坐标 (m)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Y 坐标 (m)', 'FontSize', 12, 'FontWeight', 'bold');
title('情形2飞行器椭圆轨迹与导弹攻击模拟', 'FontSize', 14, 'FontWeight', 'bold');

% 图例
legend('飞行器轨迹', '椭圆中心', '导弹起始点', '导弹轨迹', '击中点', ...
       'Location', 'best', 'FontSize', 10);

% 图形设置
axis equal;
grid on;
grid minor;
set(gca, 'FontSize', 11);

% 设置坐标轴范围，确保所有轨迹可见
x_min = min([min(X2), min(x2), 0]) - 2;
x_max = max([max(X2), max(x2), 0]) + 2;
y_min = min([min(Y2), min(y2), 0]) - 2;
y_max = max([max(Y2), max(y2), 0]) + 2;
axis([x_min x_max y_min y_max]);

% 添加文本框显示关键信息
text(0.02, 0.98, sprintf('椭圆方程: X=10+20cos(t)\n            Y=20+5sin(t)'), ...
     'Units', 'normalized', 'VerticalAlignment', 'top', ...
     'FontSize', 10, 'BackgroundColor', [0.95 0.95 0.95], ...
     'EdgeColor', [0.7 0.7 0.7]);
% 绘制距离随时间变化
figure;
plot(t, dis2, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('导弹与乙舰距离 (m)');
title('情形2导弹与乙舰距离随时间变化');
grid on;
