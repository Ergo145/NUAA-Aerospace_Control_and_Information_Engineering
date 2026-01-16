%% 阶跃响应性能指标计算与绘图脚本
% 本脚本用于分析从Simulink导出的阶跃响应数据
% 输入为阶跃响应1(t)，输出应为'yout'变量（To Workspace默认名称）
% 误差带设置为±5%

clear all; close all; clc;

%% 数据准备
% 如果您的To Workspace输出变量不是默认名称，请修改此处
% 假设时间变量为'tout'，输出变量为'yout'

% 提取数据
kp = 0.3;
ki = 0;
kd = 0.02;
out = sim("sy1.slx");
t = out.result.Time;
y = out.result.Data;

% 确保是列向量
if size(y, 1) == 1
    y = y';
end
if size(t, 1) == 1
    t = t';
end

%% 计算稳态值（假设最终达到稳态）
% 取最后10%的数据计算稳态值
n = length(y);
start_idx = floor(0.9 * n);
y_steady = mean(y(start_idx:end));

% 对于单位阶跃响应，稳态值应为1
% 但为了通用性，我们计算实际的稳态值
if abs(y_steady - 1) < 0.05  % 如果接近1，则认为是单位阶跃
    y_final = 1;
else
    y_final = y_steady;
end

%% 计算峰值和峰值时间
[y_peak, peak_idx] = max(y);
t_peak = t(peak_idx);

% 检查是否确实存在超调（峰值大于稳态值）
has_overshoot = y_peak > y_final;

%% 计算超调量
if has_overshoot
    overshoot_percent = ((y_peak - y_final) / y_final) * 100;
else
    overshoot_percent = 0;
    fprintf('系统无超调。\n');
end

%% 计算调节时间（进入并保持在±5%误差带内的时间）
% 定义误差带范围
error_band = 0.05;  % ±5%

% 计算上下界
upper_bound = y_final * (1 + error_band);
lower_bound = y_final * (1 - error_band);

% 找出首次进入误差带并之后一直保持在误差带内的时间
in_band = false;
settling_time = t(end);  % 初始化为仿真结束时间
settling_idx = n;        % 初始化为最后一个索引

for i = 1:length(y)
    if y(i) >= lower_bound && y(i) <= upper_bound
        % 检查从此点开始是否一直保持在误差带内
        remaining_in_band = true;
        for j = i:length(y)
            if y(j) < lower_bound || y(j) > upper_bound
                remaining_in_band = false;
                break;
            end
        end
        
        if remaining_in_band
            settling_time = t(i);
            settling_idx = i;
            break;
        end
    end
end

% 如果没有找到调节时间（系统未进入5%误差带）
if settling_time == t(end)
    fprintf('注意：系统在仿真时间内未进入±5%%误差带。\n');
    % 尝试找到首次进入误差带的时间（不要求之后一直保持）
    idx_in_band = find(y >= lower_bound & y <= upper_bound, 1);
    if ~isempty(idx_in_band)
        settling_time_approx = t(idx_in_band);
        fprintf('首次进入±5%%误差带的时间约为: %.4f 秒\n', settling_time_approx);
        settling_time = settling_time_approx;
        settling_idx = idx_in_band;
    end
end

%% 计算上升时间（从10%到90%稳态值的时间）
if y_final > 0
    % 找到首次达到10%稳态值的时间
    idx_10 = find(y >= 0.1 * y_final, 1);
    if ~isempty(idx_10)
        t_10 = t(idx_10);
    else
        t_10 = NaN;
    end
    
    % 找到首次达到90%稳态值的时间
    idx_90 = find(y >= 0.9 * y_final, 1);
    if ~isempty(idx_90)
        t_90 = t(idx_90);
    else
        t_90 = NaN;
    end
    
    if ~isnan(t_10) && ~isnan(t_90)
        rise_time = t_90 - t_10;
    else
        rise_time = NaN;
    end
else
    rise_time = NaN;
end

%% 计算延迟时间（达到50%稳态值的时间）
if y_final > 0
    idx_50 = find(y >= 0.5 * y_final, 1);
    if ~isempty(idx_50)
        delay_time = t(idx_50);
    else
        delay_time = NaN;
    end
else
    delay_time = NaN;
end

%% 输出性能指标结果
fprintf('========== 阶跃响应性能指标 (±5%%误差带) ==========\n');
fprintf('稳态值: %.4f\n', y_final);
fprintf('峰值: %.4f\n', y_peak);
fprintf('峰值时间: %.4f 秒\n', t_peak);
fprintf('超调量: %.2f%%\n', overshoot_percent);
fprintf('调节时间(±5%%): %.4f 秒\n', settling_time);
if ~isnan(rise_time)
    fprintf('上升时间(10%%-90%%): %.4f 秒\n', rise_time);
end
if ~isnan(delay_time)
    fprintf('延迟时间(0-50%%): %.4f 秒\n', delay_time);
end
fprintf('================================================\n');

%% 绘制阶跃响应图形
figure('Position', [100, 100, 1200, 600]);

% 主响应曲线
subplot(1, 2, 1);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;

% 绘制稳态值参考线
plot([t(1), t(end)], [y_final, y_final], 'k--', 'LineWidth', 1.5, 'Color', [0.5, 0.5, 0.5]);

% 标记峰值点
if has_overshoot
    plot(t_peak, y_peak, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    text(t_peak, y_peak, sprintf('  峰值: %.3f\n  时间: %.3fs', y_peak, t_peak), ...
        'VerticalAlignment', 'bottom', 'FontSize', 10);
end

% 标记调节时间点
if settling_time < t(end) && settling_idx <= n
    plot(settling_time, y(settling_idx), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    text(settling_time, y(settling_idx), ...
        sprintf('  调节时间: %.3fs', settling_time), ...
        'VerticalAlignment', 'top', 'FontSize', 10);
    
    % 绘制误差带
    fill_x = [t(1), t(end), t(end), t(1)];
    fill_y = [upper_bound, upper_bound, lower_bound, lower_bound];
    fill(fill_x, fill_y, [0.9, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
end

% 绘制上升时间区域
if ~isnan(rise_time)
    plot([t_10, t_10], [0, 0.1*y_final], 'm--', 'LineWidth', 1.5);
    plot([t_90, t_90], [0, 0.9*y_final], 'm--', 'LineWidth', 1.5);
    plot([t_10, t_90], [0.1*y_final, 0.9*y_final], 'm-', 'LineWidth', 2);
end

% % 绘制延迟时间点
% if ~isnan(delay_time)
%     plot(delay_time, 0.5*y_final, 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
%     text(delay_time, 0.5*y_final, ...
%         sprintf('  延迟时间: %.3fs', delay_time), ...
%         'VerticalAlignment', 'bottom', 'FontSize', 10);
% end

grid on;
xlabel('时间 (秒)', 'FontSize', 12);
ylabel('响应幅值', 'FontSize', 12);
title(sprintf('阶跃响应曲线 (误差带: ±5%%, 超调量: %.2f%%)', overshoot_percent), 'FontSize', 14);
legend_items = {'阶跃响应', '稳态值', '峰值点', '调节时间点', '±5%误差带', '上升时间', '延迟时间'};
% 只显示实际存在的图例项
legend_labels = {};
if has_overshoot
    legend_labels{end+1} = '阶跃响应';
end
legend_labels{end+1} = '稳态值';
if has_overshoot
    legend_labels{end+1} = '峰值点';
end
if settling_time < t(end) && settling_idx <= n
    legend_labels{end+1} = '调节时间点';
    legend_labels{end+1} = '±5%误差带';
end
if ~isnan(rise_time)
    legend_labels{end+1} = '上升时间';
end
if ~isnan(delay_time)
    legend_labels{end+1} = '延迟时间';
end
legend(legend_labels, 'Location', 'best');

% 绘制性能指标示意图
subplot(1, 2, 2);
% 创建一个简单的示意图
axis([0, 10, 0, 10]);
hold on;

% 绘制示意图中的响应曲线
x_schematic = linspace(0, 10, 100);
y_schematic = 5 * (1 - exp(-x_schematic/2) .* (1 + 0.5*x_schematic));
plot(x_schematic, y_schematic, 'b-', 'LineWidth', 2);

% 标注各个性能指标
text(2, 7, sprintf('峰值时间: %.3f s', t_peak), 'FontSize', 11, 'BackgroundColor', 'w');
text(4, 5, sprintf('超调量: %.2f%%', overshoot_percent), 'FontSize', 11, 'BackgroundColor', 'w');
text(6, 3, sprintf('调节时间: %.3f s', settling_time), 'FontSize', 11, 'BackgroundColor', 'w');

% 添加说明文本
text(5, 9, '性能指标示意图 (±5%误差带)', 'FontSize', 12, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');

% 添加表格显示结果
annotation('textbox', [0.75, 0.15, 0.2, 0.25], 'String', ...
    {sprintf('稳态值: %.4f', y_final), ...
     sprintf('峰值: %.4f', y_peak), ...
     sprintf('峰值时间: %.4f s', t_peak), ...
     sprintf('超调量: %.2f%%', overshoot_percent), ...
     sprintf('调节时间: %.4f s', settling_time), ...
     sprintf('上升时间: %.4f s', rise_time), ...
     sprintf('延迟时间: %.4f s', delay_time)}, ...
    'FontSize', 10, 'BackgroundColor', 'w', 'EdgeColor', 'k');

grid on;
xlabel('时间');
ylabel('幅值');
title('性能指标示意图 (±5%误差带)');

%% 保存结果
% 将结果保存到结构体中便于后续使用
performance_metrics.steady_state = y_final;
performance_metrics.peak_value = y_peak;
performance_metrics.peak_time = t_peak;
performance_metrics.overshoot_percent = overshoot_percent;
performance_metrics.settling_time = settling_time;
performance_metrics.settling_error_band = error_band;
performance_metrics.rise_time = rise_time;
performance_metrics.delay_time = delay_time;

% 保存图形
saveas(gcf, 'step_response_analysis_5percent.png');
fprintf('\n图形已保存为: step_response_analysis_5percent.png\n');
fprintf('性能指标已保存到变量: performance_metrics\n');

%% 可选：绘制更多分析图形
figure('Position', [100, 100, 1000, 800]);

% 1. 主要响应曲线
subplot(2, 2, 1);
plot(t, y, 'b-', 'LineWidth', 2);
hold on;
plot([t(1), t(end)], [y_final, y_final], 'k--', 'LineWidth', 1.5);
plot([t(1), t(end)], [upper_bound, upper_bound], 'g--', 'LineWidth', 1);
plot([t(1), t(end)], [lower_bound, lower_bound], 'g--', 'LineWidth', 1);
fill([t(1), t(end), t(end), t(1)], [upper_bound, upper_bound, lower_bound, lower_bound], ...
    [0.9, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
if has_overshoot
    plot(t_peak, y_peak, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
if settling_time < t(end) && settling_idx <= n
    plot(settling_time, y(settling_idx), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
end
grid on;
xlabel('时间 (秒)');
ylabel('响应幅值');
title('阶跃响应 (±5%误差带)');
legend('响应', '稳态值', '误差带上限', '误差带下限', '误差带', '峰值', '调节时间点', 'Location', 'best');

% 2. 响应局部放大（前25%时间）
subplot(2, 2, 2);
time_limit = t(floor(0.25 * n));
idx_limit = find(t <= time_limit, 1, 'last');
plot(t(1:idx_limit), y(1:idx_limit), 'b-', 'LineWidth', 2);
hold on;
plot([t(1), t(idx_limit)], [y_final, y_final], 'k--', 'LineWidth', 1.5);
if has_overshoot && t_peak <= time_limit
    plot(t_peak, y_peak, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
end
grid on;
xlabel('时间 (秒)');
ylabel('响应幅值');
title('响应初始阶段');

% 3. 响应局部放大（最后25%时间）
subplot(2, 2, 3);
start_idx_late = floor(0.75 * n);
plot(t(start_idx_late:end), y(start_idx_late:end), 'b-', 'LineWidth', 2);
hold on;
plot([t(start_idx_late), t(end)], [y_final, y_final], 'k--', 'LineWidth', 1.5);
plot([t(start_idx_late), t(end)], [upper_bound, upper_bound], 'g--', 'LineWidth', 1);
plot([t(start_idx_late), t(end)], [lower_bound, lower_bound], 'g--', 'LineWidth', 1);
fill([t(start_idx_late), t(end), t(end), t(start_idx_late)], ...
    [upper_bound, upper_bound, lower_bound, lower_bound], ...
    [0.9, 0.9, 0.9], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
grid on;
xlabel('时间 (秒)');
ylabel('响应幅值');
title('响应稳态阶段');

% 4. 响应偏差（相对于稳态值）
subplot(2, 2, 4);
error = y - y_final;
plot(t, error, 'r-', 'LineWidth', 1.5);
hold on;
plot([t(1), t(end)], [0, 0], 'k--', 'LineWidth', 1);
plot([t(1), t(end)], [error_band*y_final, error_band*y_final], 'g--', 'LineWidth', 1);
plot([t(1), t(end)], [-error_band*y_final, -error_band*y_final], 'g--', 'LineWidth', 1);
grid on;
xlabel('时间 (秒)');
ylabel('偏差');
title('响应偏差 (±5%误差带)');
legend('偏差', '零线', '误差带上限', '误差带下限', 'Location', 'best');

sgtitle('阶跃响应详细分析 (±5%误差带)');

% 保存详细分析图
saveas(gcf, 'step_response_detailed_analysis_5percent.png');
fprintf('详细分析图已保存为: step_response_detailed_analysis_5percent.png\n');