function Cen = p2p(Cen,Wen_n)

global deltat 
%位置矩阵的更新
Cen(1,1)=Cen(1,1)+deltat*(Wen_n(3,1)*Cen(2,1)-Wen_n(2,1)*Cen(3,1));
Cen(1,2)=Cen(1,2)+deltat*(Wen_n(3,1)*Cen(2,2)-Wen_n(2,1)*Cen(3,2));
Cen(1,3)=Cen(1,3)+deltat*(Wen_n(3,1)*Cen(2,3)-Wen_n(2,1)*Cen(3,3));
Cen(2,1)=Cen(2,1)+deltat*(-Wen_n(3,1)*Cen(1,1)+Wen_n(1,1)*Cen(3,1));
Cen(2,2)=Cen(2,2)+deltat*(-Wen_n(3,1)*Cen(1,2)+Wen_n(1,1)*Cen(3,2));
Cen(2,3)=Cen(2,3)+deltat*(-Wen_n(3,1)*Cen(1,3)+Wen_n(1,1)*Cen(3,3));
Cen(3,1)=Cen(3,1)+deltat*(Wen_n(2,1)*Cen(1,1)-Wen_n(1,1)*Cen(2,1));
Cen(3,2)=Cen(3,2)+deltat*(Wen_n(2,1)*Cen(1,2)-Wen_n(1,1)*Cen(2,2));
Cen(3,3)=Cen(3,3)+deltat*(Wen_n(2,1)*Cen(1,3)-Wen_n(1,1)*Cen(2,3));

