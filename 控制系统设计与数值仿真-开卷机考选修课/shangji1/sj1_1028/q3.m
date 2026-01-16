clc;clear;
tspan = [0,10];
x0 = [5,-6];
h = 0.01;
x=[];
t = 0:h:tspan(2);
%%仿真
x = [x;x0];
max = length(0:h:tspan(2));
for i = 2:max
    dx1 = -x(i-1,1) + x(i-1,2);
    dx2 = -x(i-1,1) - 3*x(i-1,2) - x(i-1,2)^3 + 5*sin(t(i));
    x1 = x(i-1,1) + dx1*h;
    x2 = x(i-1,2) + dx2*h;
    x = [x;x1,x2];
end
y(:,1) =x(:,1) + 2*x(:,2);
y(:,2) =3*x(:,1) - x(:,2);

figure(1);
hold on;
grid on;

plot(t,y(:,1),"r","LineWidth",2);
plot(t,y(:,2),"b","LineWidth",2);
legend("y1","y2");
xlabel("t");ylabel("y1 and y2");

figure(2);
subplot(2,1,1);
hold on;
grid on;
plot(t,y(:,1),"b","LineWidth",2);
xlabel("t");ylabel("y1");
subplot(2,1,2);
hold on;
grid on;
plot(t,y(:,2),"b","LineWidth",2);
xlabel("t");ylabel("y2");
