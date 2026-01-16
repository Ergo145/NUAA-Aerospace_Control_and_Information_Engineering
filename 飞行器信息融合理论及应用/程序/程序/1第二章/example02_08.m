%example 2-7
%比较正态分布密度函数随其标准差的变化趋势

clear all
close all

sigma1 = 0.5;
sigma2 = 2;
mu = 5;
x=(0:0.05:10)';
y1 = 1/sqrt(2*pi)/sigma1*exp(-1/2/sigma1^2*(x-mu).^2);
y2 = 1/sqrt(2*pi)/sigma2*exp(-1/2/sigma2^2*(x-mu).^2);
plot(x,y1,'ro-',x,y2,'b*-');
xlabel('\it\fontname{Times New Roman}x'),ylabel('\it\fontname{Times New Roman}f_X\rm(\it\fontname{Times New Roman}x\rm)');
legend('\sigma=0.5','\sigma=2')