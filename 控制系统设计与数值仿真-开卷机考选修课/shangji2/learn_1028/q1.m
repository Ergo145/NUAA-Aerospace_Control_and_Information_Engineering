clc;clear;

%% 参数设置
m       = 1;
f       = 5;
kp      = 1;
ki      = 0.1;
kd      = 0.5;
tmax    = 100;
dt      = 0.01;
t       = 0:dt:tmax;
% x = [eta,x,v];
X       = [0,10,10]; 
x       = [];
u       =[];
%% 仿真程序
for i = 1:length(t)
    d0x =  X(2);
    d1x =  X(3);
    df1x = X(1); 
    u_in = -kp*d0x - kd*d1x - ki*df1x;
    u = [u;u_in];
    dX1 = pid(X,u_in,f,m);
    dX2 = pid(X + 0.5*dt*dX1,u_in,f,m);
    dX3 = pid(X + 0.5*dt*dX2,u_in,f,m);
    dX4 = pid(X + dt*dX3,u_in,f,m);

    X = X + (dX1 + 2*dX2 + 2*dX3 + dX4)*dt/6;
    x = [x;X];
end
figure;
subplot(2,2,1);
plot(t,x(:,1),"b-",LineWidth=1);
hold on;
grid on;
xlabel("t");ylabel("eta");
subplot(2,2,2);
plot(t,x(:,2),"b-",LineWidth=1);
hold on;
grid on;
xlabel("t");ylabel("x");
subplot(2,2,3);
plot(t,x(:,3),"b-",LineWidth=1);
hold on;
grid on;
xlabel("t");ylabel("v");
subplot(2,2,4);
plot(t,u,"b-",LineWidth=1);
hold on;
grid on;
xlabel("t");ylabel("u");

function dX = pid(X,u,f,m)
    d0x = X(2);
    d1x = X(3);
    d2x = ( u+f ) / m;
    dX = [d0x,d1x,d2x];
end