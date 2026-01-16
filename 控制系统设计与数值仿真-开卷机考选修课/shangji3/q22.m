clc;clear;close all;
m1 = 1;m2 = 1;l = 1;g = 9.8;
A = [0,0,1,0;
     0,0,0,1;
     0,g,0,0;
     0,2*g,0,0];
B = [0,0,1,1]';
p = [-1-1i,-1+1i,-1+2i,-1-2i];
K = place(A,B,p);
dt = 0.01; t =0:0.01:50;
solution = [];
X = [1;deg2rad(35);0.2;deg2rad(2)];
for i = 1:length(t)
    dX1 = cal_dX(X             ,A,B,K);
    dX2 = cal_dX(X + 0.5*dt*dX1,A,B,K);
    dX3 = cal_dX(X + 0.5*dt*dX2,A,B,K);
    dX4 = cal_dX(X + 0.5*dt*dX3,A,B,K);

    X = X + dt/6*(dX1 + 2*dX2 + 2*dX3 +dX4);
    solution = [solution,X];
end
figure;
hold on;grid on;
plot(t,solution(1,:),"LineWidth",2);
plot(t,solution(2,:),"LineWidth",2);
xlabel("t(s)");legend("x","theta");

function dX = cal_dX(X,A,B,K)
    u = -K*X;
    dX = A*X + B*u;
end