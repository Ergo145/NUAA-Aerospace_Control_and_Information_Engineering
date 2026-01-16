% example 2-16
% 功率谱密度求解

clear all
close all

k = 1;
a1 = 0.5;
a2 = 5;

t1 = -10:0.05:-0.05;
t2 = 0:0.05:10;
t = [t1,t2];

y11 = k*exp(a1*t1);
y12 = k*exp(-a1*t2);
y1 = [y11,y12];

y21 = k*exp(a2*t1);
y22 = k*exp(-a2*t2);
y2 = [y21,y22];

omega = -10:0.05:10;

phi_y1 = 2*k*a1./(omega.^2+a1^2);
phi_y2 = 2*k*a2./(omega.^2+a2^2);

figure(1)
plot(t,y1,'r--',t,y2,'bo-')
xlabel('\it\fontname{Times New Roman}t')
ylabel('\it\fontname{Times New Roman}R_x_x\rm(\tau)')
legend('\it\fontname{Times New Roman}a=0.5','\it\fontname{Times New Roman}a=5')

figure(2)
plot(omega,phi_y1,'r--',omega,phi_y2,'bo-')
xlabel('\fontname{Times New Roman}\omega')
ylabel('\fontname{Times New Roman}\Phi_x_x(\omega)')
legend('\it\fontname{Times New Roman}a=0.5','\it\fontname{Times New Roman}a=5')