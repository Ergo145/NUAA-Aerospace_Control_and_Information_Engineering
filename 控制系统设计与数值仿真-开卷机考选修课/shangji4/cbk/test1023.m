clear,clc,close all


d2r = pi/180;

%%%%仿真参数初始化
global m omega 

omega = 1;
m = 1;

fsi  = 30*d2r;
a = 5;
f0 = 6;
x0 = 0;
dx0 = 0;
y0 = 0;
dy0 = 0;
ddy0 = 0;

A =[0 ,1,0,0,0;
    0,0,1,0,0;
    0,-omega^2,0,1,0;
    0,0,0,0,1;
    0,0,0,0,0
    ];
B = [0,0,0,0,1/m]';


%%%%%求解控制器增益
p =[-1+i,-1-i,-1+2i,-1-2i,-1] 
K =place(A,B,p);

state = [y0 ;dy0;ddy0;x0;dx0];


%%%初始化时间与输出
stateout =[state];
uout = [];
fout = [];
t0 = 0;
tf = 50;
dt = 0.01;

for t = t0:dt:tf
    u = -K*state;
    f = f0 + a*sin(omega*t+fsi);

    %%%龙格库塔
    ke1 = state1023(state,u,t,f);
    ke2 = state1023(state+ke1*0.5*dt,u,t+0.5*dt,f);
    ke3 = state1023(state+ke2*0.5*dt,u,t+0.5*dt,f);
    ke4 = state1023(state+ke3*dt,u,t+dt,f);

    state = state+1/6*(ke1+2*ke2+2*ke3+ke4)*dt;

    stateout = [stateout,state];
    fout = [fout,f];
    uout = [uout,u];
  
end

tout = t0:dt:tf;
uout = [uout,uout(end)];
tout = [tout,tout(end)];
fout = [fout,fout(end)];
subplot(5,1,1),
plot(tout,stateout(1,:),'LineWidth',2);
ylabel('Z1')
subplot(5,1,2),
plot(tout,stateout(4,:),'LineWidth',2);
ylabel('X')
subplot(5,1,3),
plot(tout,stateout(5,:),'LineWidth',2);
ylabel('V')
subplot(5,1,4),
plot(tout,uout,LineWidth=2);
hold on,
plot(tout,fout,LineWidth=2);
legend('u','f');
ylabel('u and f')
subplot(5,1,5),
plot(tout,uout+fout,'LineWidth',2);
ylabel('u+f')












