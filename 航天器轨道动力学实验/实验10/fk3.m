%% 复刻图 13.9：二体问题下的惯性系与旋转系对比
clc; clear; close all;

% --- 1. 参数设置 ---
% 注意：图13.9 是二体问题，所以 mu = 0 (只考虑中心天体)
mu_2body = 0; 
t_span = [0, 12]; 

% 初始条件：产生一个椭圆轨道
% 在惯性系下这是一个椭圆，但在旋转系下观察则不同
ecc = 0.6; % 偏心率
r_p = 0.5; % 近地点距离
v_p = sqrt(1*(1+ecc)/r_p); % 活力公式计算近地点速度 (mu=1)
Y0 = [r_p; 0; 0; v_p]; 

% --- 2. 积分 (旋转系方程，但 mu=0) ---
options = odeset('RelTol', 1e-9, 'AbsTol', 1e-9);
[t, Y] = ode45(@(t,y) cr3bp_func(t,y,0), t_span, Y0, options);

x_rot = Y(:,1); y_rot = Y(:,2);

% --- 3. 变换到惯性系 ---
x_in = x_rot .* cos(t) - y_rot .* sin(t);
y_in = x_rot .* sin(t) + y_rot .* cos(t);

% --- 4. 绘图 (仿照 PDF 图 13.9 样式) ---
figure('Name', '复刻图13.9：二体问题综合对比', 'Color', 'w');
hold on; axis equal; grid on; box on;

% 绘制中心天体 (Earth)
h_earth = plot(0, 0, 'k.', 'MarkerSize', 25); 

% 绘制惯性系轨迹 (椭圆)
h_in = plot(x_in, y_in, 'k-', 'LineWidth', 1.5); 

% 绘制旋转系轨迹 (花瓣/螺旋)
h_rot = plot(x_rot, y_rot, 'b--', 'LineWidth', 1.2); 

% 标注图例
legend([h_earth, h_in, h_rot], ...
    {'Center Body (中心天体)', 'Inertial Frame Traj. (惯性系-椭圆)', 'Rotating Frame Traj. (旋转系-螺旋)'}, ...
    'Location', 'northeast');

title('图 13.9 二体问题：惯性系 vs 旋转系轨迹');
xlabel('X / x'); ylabel('Y / y');

% 标注向量箭头 (仿照图中样式)
% 在轨迹末端添加箭头
quiver(x_in(end), y_in(end), x_in(end)-x_in(end-5), y_in(end)-y_in(end-5), ...
    0, 'k', 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility', 'off');
quiver(x_rot(end), y_rot(end), x_rot(end)-x_rot(end-5), y_rot(end)-y_rot(end-5), ...
    0, 'b', 'LineWidth', 2, 'MaxHeadSize', 2, 'HandleVisibility', 'off');

% --- 复用之前的动力学函数 ---
function dY = cr3bp_func(~, Y, mu)
    x = Y(1); y = Y(2); vx = Y(3); vy = Y(4);
    r1 = sqrt((x+mu)^2 + y^2);
    r2 = sqrt((x-(1-mu))^2 + y^2);
    ax = 2*vy + x - (1-mu)*(x+mu)/r1^3 - mu*(x-(1-mu))/r2^3;
    ay = -2*vx + y - (1-mu)*y/r1^3 - mu*y/r2^3;
    dY = [vx; vy; ax; ay];
end