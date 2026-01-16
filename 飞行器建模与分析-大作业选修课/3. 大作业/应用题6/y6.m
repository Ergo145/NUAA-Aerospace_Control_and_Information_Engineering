% Lorenz系统参数
beta = 8/3;
rho = 10;
sigma = 28;

% 初始条件
epsilon = 1e-10;
x0 = [0; 0; epsilon]; % x1(0)=0, x2(0)=0, x3(0)=epsilon

% 时间范围（可以根据需要调整）
tspan = [0 50];

% 定义Lorenz系统的微分方程
lorenz_system = @(t, x) [
    -beta * x(1) + x(2) * x(3);   % dx1/dt
    -rho * x(2) + rho * x(3);     % dx2/dt
    -x(1) * x(2) + sigma * x(2) - x(3)  % dx3/dt
];

% 使用ode45求解
[t, x] = ode45(lorenz_system, tspan, x0);

% 绘制相图（x1-x2-x3）
figure;
plot3(x(:,1), x(:,2), x(:,3));
xlabel('x_1');
ylabel('x_2');
zlabel('x_3');
title('Lorenz系统相图');
grid on;

% 绘制状态变量随时间变化
figure;
plot(t, x(:,1));
ylabel('x_1');
title('状态变量随时间变化');
figure;
plot(t, x(:,2));
ylabel('x_2');
figure;
plot(t, x(:,3));
ylabel('x_3');
xlabel('时间 t');