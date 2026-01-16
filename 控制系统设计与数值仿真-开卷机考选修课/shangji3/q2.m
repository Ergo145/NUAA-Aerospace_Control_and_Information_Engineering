clc;clear;

%% 参数设置
t_max   = 25;
%X = [x,v,theta,w]
X       = [1,0.2,deg2rad(35),deg2rad(2)];
dt      = 0.01;
t       = 0:dt:t_max;
m1      = 1;
m2      = 1;
l       = 1;
g       = 9.8;

%pid
A = [0,1,0,0;
    0,0,m2*g/m1,0;
    0,0,0,1;
    0,0,(m1+m2)*g/(m1*l),0
    ];
B = [0; 1/m1; 0; 1/(m1*l)];
p = [-2+2i, -2-2i, -3+2i,-3-2i];
k = place(A,B,p);
			

Mout=[];
Xout=[];
%% 仿真
for m=1:length(t)
    
    u = -k*X';

    dX1 = individual(X,u,m1,m2,l,g);
    dX2 = individual(X+0.5*dt*dX1,u,m1,m2,l,g);
    dX3 = individual(X+0.5*dt*dX2,u,m1,m2,l,g);
    dX4 = individual(X+0.5*dt*dX3,u,m1,m2,l,g);

    X = X + (dX1 + 2*dX2 + 2*dX3 + dX4)*dt/6;
    Xout = [Xout;X];
end

figure(1);
hold on;
grid on;
plot(t,Xout(:,1),"b-",LineWidth=2);
plot(t,Xout(:,3),"r-",LineWidth=2);
legend("x","\theta");

%% 计算函数
function dX = individual(X,u,m1,m2,l,g)
    x = X(1);
    v = X(2);
    theta = X(3);
    w = X(4);

    % 公共分母
    denominator = m1 + m2 * sin(theta)^2;
    
    % 计算 x 方向的加速度
    dxdt = v;
    dvdt = (m2 * g * cos(theta) * sin(theta) - m2 * l * w^2 * sin(theta)) / denominator ...
          + u / denominator ;
    
    % 计算 theta 方向的角加速度
    dthetadt = w;
    dwdt = ((m1 + m2) * g * sin(theta) - m2 * l * w^2 * cos(theta) * sin(theta)) / (denominator * l) ...
          + (cos(theta) / (denominator * l)) * u;
    
    % 返回状态导数
    dX = [dxdt, dvdt, dthetadt, dwdt];
    
end