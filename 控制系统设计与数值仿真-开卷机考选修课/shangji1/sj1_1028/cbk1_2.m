clear all;clc;close all;
statey=[0;0;0;0];
[t,z]=ode45(@myode2,[0,20],statey);
z2=z(:,2);
z1=z(:,1);
y1=-z2+3*z1;
plot(t,y1,'r','linewidth',2);
xlabel('时间');
ylabel('Y');
grid on;

function zdot=myode2(t,z)
z1=z(1);
z2=z(2);
z3=z(3);
z4=z(4);
z1dot=z(2);
z2dot=z(3);
z3dot=z(4);
z4dot=5*exp(-0.5*t)*cos(t)+6*sin(t+pi/3)-6*z(4)-18*z(3)-24*z(2)-16*z(1);
zdot=[z1dot;z2dot;z3dot;z4dot];
end

