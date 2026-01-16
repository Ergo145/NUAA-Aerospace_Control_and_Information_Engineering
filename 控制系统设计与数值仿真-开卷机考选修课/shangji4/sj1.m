%% 参数设置
clc;clear;close all;
W = 1;
m = 1;
a = 5;
phi = deg2rad(30);
f0 = 6;
Z = [0;0;0;0;0];
A = [0, 1,0,0,0;
     0, 0,1,0,0;
     0,-1,0,1,0;
     0, 0,0,0,1;
     0, 0,0,0,0];
B = [0,0,0,0,1/m]';
p = [-1+1i,-1-1i,-2+1i,-2-1i,-1];
K = place(A,B,p);
tf = 50;
dt = 0.01;
t = 0:dt:tf;
uout = [];
fout = [];
Zout = [];

for k = 1:length(t)
    u = -K*Z;
    f = f0 + a*sin(W*t(k)+phi);
    
    dZ1 = cal_dZ(Z,A,B,u,f);
    dZ2 = cal_dZ(Z + 0.5*dt*dZ1,A,B,u,f);
    dZ3 = cal_dZ(Z + 0.5*dt*dZ2,A,B,u,f);
    dZ4 = cal_dZ(Z + dt*dZ3,A,B,u,f);

    Z = Z + dt*(dZ1 + 2*dZ2 + 2*dZ3 + dZ4)/6;
    uout = [uout,u];
    fout = [fout,f];
    Zout = [Zout,Z];
end

figure(1);
subplot(3,1,1);
hold on; grid on;
plot(t,Zout(1,:),"LineWidth",2);
ylabel("z_1");
subplot(3,1,2);
hold on; grid on;
plot(t,Zout(4,:),"LineWidth",2);
ylabel("x");
subplot(3,1,3);
hold on; grid on;
plot(t,Zout(5,:),"LineWidth",2);
ylabel("v");
figure(2);
subplot(2,1,1);
hold on; grid on;
plot(t,uout,"r","LineWidth",2);
plot(t,fout,"black","LineWidth",2);
ylabel("u and f");legend("u","f");
subplot(2,1,2);
hold on; grid on;
plot(t,uout+fout,"LineWidth",2);
ylabel("u + f");

function dZ = cal_dZ(Z,A,B,u,f)
    dZ = A*Z + B*(u+f);
end