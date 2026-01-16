% 清理工作区和命令行
clear;
clc;
close all;

% --- 实验参数 ---
D = 1; % 平行线之间的距离
L = 0.8; % 针的长度 (确保 L <= D)
num_trials = 10000; % 投掷针的次数

% --- 模拟过程 ---
intersect_count = 0; % 记录相交的次数

% 初始化用于绘图的数组
needle_starts_x = zeros(num_trials, 1);
needle_starts_y = zeros(num_trials, 1);
needle_ends_x = zeros(num_trials, 1);
needle_ends_y = zeros(num_trials, 1);
intersect_flags = false(num_trials, 1);

% 循环进行每次投掷
for i = 1:num_trials
    % 随机生成针的中心点y坐标 (0 到 D 之间)
    % 我们假设第一条平行线在 y=0，第二条在 y=D
    center_y = rand() * D;

    % 随机生成针与x轴的夹角 (0 到 pi 之间)
    angle = rand() * pi;

    % 计算针的两个端点的y坐标
    y1 = center_y - (L/2) * sin(angle);
    y2 = center_y + (L/2) * sin(angle);

    % 判断针是否与平行线相交
    % 相交的条件是两个端点分跨0和D这两条线
    % 或者更简单地，如果 y1 和 y2 之间包含了整数倍的 D (在这个简化模型中是0或D)
    % 或者说，如果 y1 和 y2 的符号不同，或者一个小于0一个大于D
    
    % 这里我们简化处理：只需判断针是否跨越了0或D这条线 (即是否跨越了一个平行线)
    % 如果针的最低点低于0或者最高点高于D，则认为它与平行线相交
    % (因为我们只考虑单条平行线间隔内的投掷，所以只需要判断是否跨越了 y=0 或 y=D)
    if y1 < 0 || y2 > D
        intersect_count = intersect_count + 1;
        intersect_flags(i) = true;
    end
    
    % 记录绘图数据 (x坐标可以随意设定，我们用 i 来错开显示)
    % 为了更好地可视化，我们可以将所有的针都集中在一个 D 的宽度内
    needle_starts_x(i) = i; % 或者使用一个固定的x值，例如0
    needle_starts_y(i) = y1;
    needle_ends_x(i) = i + L * cos(angle); % x坐标可以根据角度稍微变化
    needle_ends_y(i) = y2;
end

% --- 计算估计的圆周率 ---
if intersect_count > 0
    estimated_pi = (2 * L * num_trials) / (D * intersect_count);
else
    estimated_pi = NaN; % 如果没有相交，则无法估算
end

% --- 结果显示 ---
fprintf('总投掷次数：%d\n', num_trials);
fprintf('相交次数：%d\n', intersect_count);
fprintf('相交概率 P = 相交次数 / 总投掷次数 = %.4f\n', intersect_count / num_trials);
fprintf('理论相交概率 P_theory = 2L / (pi*D) = %.4f\n', (2 * L) / (pi * D));
fprintf('估计的圆周率 (2 * L * num_trials) / (D * intersect_count) = %.4f\n', estimated_pi);
fprintf('真实的圆周率 pi = %.4f\n', pi);

% --- 可视化 ---
figure;
hold on;

% 绘制平行线
line([0, num_trials+L], [0, 0], 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 1.5); % 第一条平行线
line([0, num_trials+L], [D, D], 'Color', 'blue', 'LineStyle', '--', 'LineWidth', 1.5); % 第二条平行线

% % 绘制针 (相交的用红色，不相交的用灰色)
% for i = 1:num_trials
%     if intersect_flags(i)
%         plot([i, i + L * cos(angle)], [needle_starts_y(i), needle_ends_y(i)], 'r-', 'LineWidth', 1);
%     else
%         plot([i, i + L * cos(angle)], [needle_starts_y(i), needle_ends_y(i)], 'Color', [0.7 0.7 0.7], 'LineWidth', 0.5);
%     end
% end
% 
% title('布丰投针实验可视化');
% xlabel('投掷次序 (X轴表示针的编号，Y轴表示位置)');
% ylabel('Y坐标');
% ylim([-L, D+L]); % 调整Y轴范围，确保看到超出边界的针
% xlim([0, num_trials+L]); % 调整X轴范围
grid on;
hold off;