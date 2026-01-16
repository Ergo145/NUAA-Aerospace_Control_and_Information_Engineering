clc; clear; close all;

%% 参数
m0 = 50;     % 小车质量
m = 5;       % 吊物质量
l = 1;       % 绳长
g = 9.8;     % 重力加速度

%% 仿真时间
tspan = [0 10];  % 模拟 10 秒

%% 初值 [x, dx, theta, dtheta]
y0 = [0; 0; 0.1; 0];  % 初始摆角 0.1 rad，其余为 0

%% 外力函数 (阶跃力: t>=0 时 F=1)
F_func = @(t) (t >= 0) * 1;  

%% ========== 复杂模型 ==========
ode_fun_complex = @(t,y) crane_complex(t,y,m0,m,l,g,F_func);

[t1,y1] = ode45(ode_fun_complex,tspan,y0);

% 计算加速度 & 角加速度
ddx1 = zeros(size(t1));
ddtheta1 = zeros(size(t1));
for i=1:length(t1)
    dy = crane_complex(t1(i),y1(i,:)',m0,m,l,g,F_func);
    ddx1(i) = dy(2);
    ddtheta1(i) = dy(4);
end

%% ========== 简化模型 ==========
ode_fun_simple = @(t,y) crane_simple(t,y,m0,m,l,g,F_func);

[t2,y2] = ode45(ode_fun_simple,tspan,y0);

ddx2 = zeros(size(t2));
ddtheta2 = zeros(size(t2));
for i=1:length(t2)
    dy = crane_simple(t2(i),y2(i,:)',m0,m,l,g,F_func);
    ddx2(i) = dy(2);
    ddtheta2(i) = dy(4);
end

%% ========== 绘图 ==========
figure;

% 位移
subplot(3,2,1);
plot(t1,y1(:,1),'b-',t2,y2(:,1),'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('x (m)');
legend('复杂模型','简化模型'); title('小车位移');

% 速度
subplot(3,2,3);
plot(t1,y1(:,2),'b-',t2,y2(:,2),'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('dx (m/s)');
legend('复杂模型','简化模型'); title('小车速度');

% 加速度
subplot(3,2,5);
plot(t1,ddx1,'b-',t2,ddx2,'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('ddx (m/s^2)');
legend('复杂模型','简化模型'); title('小车加速度');

% 角度
subplot(3,2,2);
plot(t1,y1(:,3),'b-',t2,y2(:,3),'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('\theta (rad)');
legend('复杂模型','简化模型'); title('吊角度');

% 角速度
subplot(3,2,4);
plot(t1,y1(:,4),'b-',t2,y2(:,4),'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('d\theta (rad/s)');
legend('复杂模型','简化模型'); title('角速度');

% 角加速度
subplot(3,2,6);
plot(t1,ddtheta1,'b-',t2,ddtheta2,'r--','LineWidth',1.2);
xlabel('t (s)'); ylabel('dd\theta (rad/s^2)');
legend('复杂模型','简化模型'); title('角加速度');

sgtitle('吊车系统复杂模型与简化模型对比');
