clear,clc,close all
global m1 
m1 = 1;
global m2
m2  =   1 ;
global  l  
l = 1;
global    g  
g  =    9.8;
d2r = pi/180;
%%初始化时间向量
t0 = 0;
tf = 100;
dt = 0.01;
tout = t0:dt:tf;
%%初始化控制器
kp1 = 10;
kp2 = 20;
kd1 = 50;
kd2 = 50;
%%初始化输出
x0 = 1;
dx0 =0.2 ;
theta0 =35*d2r ;
dtheta0 =2*d2r ;


state0 = [x0;dx0;theta0;dtheta0];
stateout =[state0];
state = state0;   %%容器变量，每次执行循环，他的值不断变化

uout =[];
Mout = [];

for t = t0:dt:tf
    x = state(1);
    dx = state(2);
    theta = state(3);
    dtheta = state(4);
    u = -kp1*x-kd1*dx;
    M = -kp2*theta-kd2*dtheta;

    ke1 =daolibaiequation(t,state,M,u);
    ke2 =daolibaiequation(t+0.5*dt,state+0.5*dt*ke1,M,u);
    ke3 =daolibaiequation(t+0.5*dt,state+0.5*dt*ke2,M,u);
    ke4 =daolibaiequation(t+dt,state+dt*ke1,M,u);
    
    state  =state +1/6*(ke1+2*ke2+2*ke3+ke4)*dt;

    stateout = [stateout,state];
    uout =[u];
    Mout = [M];

end

tout = [tout,tout(end)];
uout = [uout,uout(end)];
Mout = [Mout,Mout(end)];

figure;
plot(tout, stateout(1,:), 'r', tout, stateout(3,:), 'g', 'LineWidth', 2)
xlabel('t/s');
legend('x (m)','theta (°)')











