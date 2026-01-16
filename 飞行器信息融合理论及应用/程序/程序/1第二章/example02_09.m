% example 2-8
% 二维高斯分布画图

clear all
close all

sigma = 2;
x=-5:0.2:5;
y=-5:0.2:5;

[X,Y] = meshgrid(x,y);
Z = 1/2/pi/sigma^2*exp(-((X-0).^2+(Y-0).^2)/2/sigma^2);

surf(X,Y,Z)
xlabel('\it\fontname{Times New Roman}x')
ylabel('\it\fontname{Times New Roman}y')
zlabel('\it\fontname{Times New Roman}z')