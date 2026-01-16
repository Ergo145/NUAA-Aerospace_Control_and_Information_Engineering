% 实验二：
clc;clear;close all;
k=100;t=[0.01,0.05,0.1];
wwn=[];zz=[];
for i =1:length(t)
    num=k;
    den=conv([1,0],[t(i),1]);
    g0=tf(num,den);
    g=feedback(g0,1);
    [wn,z,p]=damp(g);
    gg(i)=g
    wwn=[wwn wn]
    zz=[zz,z]
end
step(gg(1),'-',gg(2),'--',gg(3),'-.')
s1 = stepinfo(gg(1),"SettlingTimeThreshold",0.05);
s2 = stepinfo(gg(2),"SettlingTimeThreshold",0.05);
s3 = stepinfo(gg(3),"SettlingTimeThreshold",0.05);
legend("T=0.01","T=0.05","T=0.1");
title("阶跃响应");
xlabel("t");ylabel("振幅");