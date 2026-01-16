clc;clear;
simout = load("Allstates.mat");
t     = simout.Allstates(1,:)';
R     = simout.Allstates(2,:)';
tau   = simout.Allstates(3,:)';
delta = simout.Allstates(4,:)';
V     = simout.Allstates(5,:)';
chi   = simout.Allstates(6,:)';
gamma = simout.Allstates(7,:)';
alpha = simout.Allstates(8,:)';
beta  = simout.Allstates(9,:)';
sigma = simout.Allstates(10,:)';
p     = simout.Allstates(11,:)';
q     = simout.Allstates(12,:)';
r     = simout.Allstates(13,:)';
q_bar = simout.Allstates(14,:)';
LD    = simout.Allstates(15,:)';

% 创建第一个图形窗口 - 4x3子图（前12个变量）
figure('Position', [100, 100, 1200, 900]);

% 第1行
subplot(4,3,1);
plot(t, R, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('R');
grid on;
legend('R');

subplot(4,3,2);
plot(t, tau, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\tau');
grid on;
legend('\tau');

subplot(4,3,3);
plot(t, delta, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\delta');
grid on;
legend('\delta');

% 第2行
subplot(4,3,4);
plot(t, V, 'm-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('V');
grid on;
legend('V');

subplot(4,3,5);
plot(t, chi, 'c-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\chi');
grid on;
legend('\chi');

subplot(4,3,6);
plot(t, gamma, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\gamma');
grid on;
legend('\gamma');

% 第3行
subplot(4,3,7);
plot(t, alpha, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\alpha');
grid on;
legend('\alpha');

subplot(4,3,8);
plot(t, beta, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\beta');
grid on;
legend('\beta');

subplot(4,3,9);
plot(t, sigma, 'g-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\sigma');
grid on;
legend('\sigma');

% 第4行
subplot(4,3,10);
plot(t, p, 'm-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('p');
grid on;
legend('p');

subplot(4,3,11);
plot(t, q, 'c-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('q');
grid on;
legend('q');

subplot(4,3,12);
plot(t, r, 'k-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('r');
grid on;
legend('r');

sgtitle('状态变量时间响应 (前12个变量)');

% 创建第二个图形窗口 - 2x1子图（最后2个变量）
figure('Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(t, q_bar, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('\q_bar');
grid on;
legend('\q_bar');

subplot(2,1,2);
plot(t, LD, 'r-', 'LineWidth', 1.5);
xlabel('时间 (s)'); ylabel('L/D');
grid on;
legend('L/D');

sgtitle('状态变量时间响应 (最后2个变量)');

% 调整所有图形的显示
disp('图形绘制完成！');
exportgraphics(figure(1), 'E:\Programing\MATLAB\feixingqi\shangji2\1.jpg', 'Resolution', 300);
exportgraphics(figure(2), 'E:\Programing\MATLAB\feixingqi\shangji2\2.jpg', 'Resolution', 300);