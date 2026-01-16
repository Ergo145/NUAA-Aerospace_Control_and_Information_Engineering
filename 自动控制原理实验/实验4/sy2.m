x=[6.28 12.56 15.7 18.84 20.096 ];%注意！这是个例子
f=[0.64 2.77 4.49 6.45 7 ];                                 %理论L
f1=[0.9065 3.1067 4.8 6.9271 7.1205 ];  %实验L
y=[-7.79 -20.3 -32 -53 -64.7 ];                           %理论相位差
y1=[-7.2 -19.3 -30 -50 -60 ];                               %实验相位差
subplot(2,1,1);semilogx(x,f,'v-');
xlim('manual');xlim([6 70]);
hold on
semilogx(x,f1,'r*-');
hold off
grid
subplot(2,1,2);semilogx(x,y,'v-');
xlim('manual');xlim([6 70]);
hold on
semilogx(x,y1,'r*-');
hold off
grid
