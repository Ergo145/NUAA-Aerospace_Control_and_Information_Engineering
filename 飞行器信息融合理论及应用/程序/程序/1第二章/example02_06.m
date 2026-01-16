%example 2-6
%独立随机变量和的分布

clear all
close all

T = 2;

x1 = -T:0.05:-0.05;
x2 = 0:0.05:T-0.05;
x3 = T:0.05:2*T-0.05;
x4 = 2*T:0.05:3*T-0.05;
x5 = 3*T:0.05:4*T-0.05;
x_4 = -T:0.05:3*T-0.05;
x_5 = -T:0.05:4*T-0.05;

y11 = zeros(1,T/0.05);
y12 = 1/T*ones(1,T/0.05);
y13 = zeros(1,T/0.05);
x_1 = [x1,x2,x3];
y_1 = [y11,y12,y13];

y21 = zeros(1,T/0.05);
y22 = x2/T/T;
y23 = (2*T-x3)/T/T;
y24 = zeros(1,T/0.05);
x_2 = [x1,x2,x3,x4];
y_2 = [y21,y22,y23,y24];

y31 = zeros(1,T/0.05);
y32 = x2.^2/2/T^3;
y33 = (6*T*x3-3*T^2-2*x3.^2)/2/T^3;
y34 = (3*T-x4).^2/2/T^3;
y35 = zeros(1,T/0.05);
x_3 = [x1,x2,x3,x4,x5];
y_3 = [y31,y32,y33,y34,y35];

y_4 = sqrt(3/pi)/T*exp(-3*(x_4-T).^2/T/T);
y_5 = sqrt(2/pi)/T*exp(-2*(x_5-1.5*T).^2/T/T);

figure(1)
plot(x_1,y_1,'-.');
axis([-T 2*T 0 0.6]);
xlabel('\it\fontname{Times New Roman}x'),ylabel('\it\fontname{Times New Roman}f_X\rm(\it\fontname{Times New Roman}x\rm)');
figure(2)
plot(x_2,y_2,x_4,y_4,'r--');
xlabel('\it\fontname{Times New Roman}x'),ylabel('\it\fontname{Times New Roman}f_X\rm(\it\fontname{Times New Roman}x\rm)');
legend('X1+X2','正态分布')
figure(3)
plot(x_3,y_3,x_5,y_5,'r--');
xlabel('\it\fontname{Times New Roman}x'),ylabel('\it\fontname{Times New Roman}f_X\rm(\it\fontname{Times New Roman}x\rm)');
legend('X1+X2+X3','正态分布')