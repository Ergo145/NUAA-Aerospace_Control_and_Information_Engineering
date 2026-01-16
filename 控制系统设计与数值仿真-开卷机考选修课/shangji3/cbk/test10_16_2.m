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
A = [0,1,0,0;
    0,0,m2*g/m1,0;
    0,0,0,1;
    0,0,(m1+m2)*g/(m1*l),0
    ];
B = [0; 1/m1; 0; 1/(m1*l)];
p = [-1+i, -1-i, -2+i,-2-i];
k = place(A,B,p);
%%初始化时间向量
t0 = 0;
tf = 100;
dt = 0.01;
tout = t0:dt:tf;

%%初始化输出
x0 = 1;
dx0 =0.2 ;
theta0 =35*d2r ;
dtheta0 =2*d2r ;


state0 = [x0;dx0;theta0;dtheta0];
stateout =[state0];
state = state0;   %%容器变量，每次执行循环，他的值不断变化

uout =[];


for t = t0:dt:tf
    u = -k*state;
    ke1 =daolibaiequation2(t,state,u);
    ke2 =daolibaiequation2(t+0.5*dt,state+0.5*dt*ke1,u);
    ke3 =daolibaiequation2(t+0.5*dt,state+0.5*dt*ke2,u);
    ke4 =daolibaiequation2(t+dt,state+dt*ke1,u);
    
    state  =state +1/6*(ke1+2*ke2+2*ke3+ke4)*dt;

    stateout = [stateout,state];

end

tout = [tout,tout(end)];



figure;
plot(tout, stateout(1,:), 'r', tout, stateout(3,:), 'g', 'LineWidth', 2)
xlabel('t/s');
legend('x (m)','theta (°)')










