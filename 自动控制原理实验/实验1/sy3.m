% 实验三：
clc;clear;close all;
k = [0,0.2]; 
t = [0,0.2]; 
wwn = [];zz = [];
sx=1;
for i = 1:length(t)
    num = 10;
    den = conv([1 0],[0.625 1]);
    g0 = tf(num,den);
    den1 = [k(i),0];
    Fd = tf(den1,1);
    g1= feedback(g0,Fd);
    for j = 1:length(k)
        den2 = [t(j),1]
        g2 = tf(den2,1)
        g3 = series(g1,g2)
        g4 = feedback(g3,1)
        [wn,z,p] = damp(g4);
         gg(sx) = g4
         sx=sx+1;
         wwn = [wwn wn]
         zz = [zz,z]
    end
end
step(gg(1),'-',gg(2),'--',gg(3),'-.',gg(4),':')
s1 = stepinfo(gg(1),"SettlingTimeThreshold",0.05);
s2 = stepinfo(gg(2),"SettlingTimeThreshold",0.05);
s3 = stepinfo(gg(3),"SettlingTimeThreshold",0.05);
s4 = stepinfo(gg(4),"SettlingTimeThreshold",0.05);
legend("Td=0,Kt=0","Td=0.2,Kt=0","Td=0,Kt=0.2","Td=0.2,Kt=0.2");
title("阶跃响应");
xlabel("t");ylabel("振幅");
