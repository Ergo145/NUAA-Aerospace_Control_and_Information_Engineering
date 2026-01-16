function q = wq2q(Wnb_b,q)
%四元数更新，角增量法，一阶角增量即为欧拉法
global deltat
dtheta=Wnb_b*deltat;
d2theta=norm(dtheta)^2;
dmtheta=[0 -dtheta(1) -dtheta(2) -dtheta(3);dtheta(1) 0 dtheta(3) -dtheta(2);...
         dtheta(2) -dtheta(3) 0 dtheta(1);dtheta(3) dtheta(2) -dtheta(1) 0];
%一阶算法
q=(eye(4)+0.5*dmtheta)*q;
%二阶算法
q=((1-d2theta/8)*eye(4)+0.5*dmtheta)*q;
%四元数的归一化
q=q/norm(q);