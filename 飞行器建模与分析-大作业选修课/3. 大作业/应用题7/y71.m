clc;clear;
simout = sim('y7s.slx');
t = simout.x.Time;
x = simout.x.Data;
y = simout.y.Data;
dis = simout.dis.Data;
%% 舰船
x0_target = 2; y0_target = 0;       % 乙舰初始位置 (2,0)
dt = 0.001;
t_max = 10;
t = 0:dt:t_max;
x_target = zeros(size(t));
y_target = zeros(size(t));
v_target = 2;
x_target(1) = x0_target;
y_target(1) = y0_target;
for i = 2:length(t)
    % 乙舰位置 (沿y轴匀速运动)
    x_target(i) = x0_target;
    y_target(i) = y0_target + v_target * t(i);
end
% 绘制轨迹
figure;
plot(x, y, 'r-', 'LineWidth', 2); hold on;
plot(x_target, y_target, 'b--', 'LineWidth', 2);
plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
plot(x0_target, y0_target, 'bs', 'MarkerSize', 8, 'MarkerFaceColor', 'b');
xlabel('x 坐标 (m)');
ylabel('y 坐标 (m)');
title('导弹追踪乙舰轨迹');
legend('导弹轨迹', '乙舰轨迹', '导弹起始点', '乙舰起始点', '击中点', ...
       'Location', 'northwest');
grid on;


% 绘制距离随时间变化
figure;
plot(t, dis, 'LineWidth', 2);
xlabel('时间 (s)');
ylabel('导弹与乙舰距离 (m)');
title('导弹与乙舰距离随时间变化');
grid on;