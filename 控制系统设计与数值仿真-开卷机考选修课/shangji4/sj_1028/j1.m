clc;clear;

%% 参数设置
tmax = 50;
dt = 0.01;
t= 0:dt:tmax;
m = 1;
W = 1;%rad/s
a = 5;
phi = deg2rad(30);
f0 = 6;
Z = [0;0;0;0;0];
A = [0,1,0,0,0;
     0,0,1,0,0;
     0,-W^2,0,1,0;
     0,0,0,0,1;
     0,0,0,0,0];
B = [0,0,0,0,1/m]';

p = [-1+1i,-1-1i,-2+1i,-2-1i,-1];

K = place(A,B,p);

Zout = [];
uout = [];
fout = [];
%% 仿真

for k = 1:length(t)
    f = f0 + a*sin(W*t(k) + phi);
    fout = [fout;f];
    u = -K * Z;
    uout = [uout;u];
    dZ1 = fankui(Z,A,B,u,f);
    dZ2 = fankui(Z+0.5*dt*dZ1,A,B,u,f);
    dZ3 = fankui(Z+0.5*dt*dZ2,A,B,u,f);
    dZ4 = fankui(Z+0.5*dt*dZ3,A,B,u,f);

    Z = Z + (dZ1 + 2*dZ2 + 2*dZ3 + dZ4)*dt/6;
    Zout = [Zout,Z];

end
figure(1);
subplot(3,1,1);
hold on;
grid on;
plot(t,Zout(1,:),"b-","LineWidth",2);
xlabel("t");ylabel("z_1");
subplot(3,1,2);
hold on;
grid on;
plot(t,Zout(4,:),"b-","LineWidth",2);
xlabel("t");ylabel("x");
subplot(3,1,3);
hold on;
grid on;
plot(t,Zout(5,:),"b-","LineWidth",2);
xlabel("t");ylabel("v");
figure(2);
subplot(2,1,1);
hold on;
grid on;
plot(t,uout,"r-","LineWidth",2);
plot(t,fout,"black-","LineWidth",2);
xlabel("t");ylabel("u and f");
legend("u","f");
subplot(2,1,2);
hold on;
grid on;
plot(t,uout+fout,"b-","LineWidth",2);
xlabel("t");ylabel("u + f");
%% 函数
function dZ = fankui(Z,A,B,u,f)
    dZ = A*Z + B*(u + f);
end