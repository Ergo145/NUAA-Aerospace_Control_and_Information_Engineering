%标准滤波估计函数
function [xestk,K,Pestk] = Update(Pprek,Pvnewk,Pxzk,xprek,vnewk)
%
%更新过程（滤波估计）estimator
%滤波增益K:
%K=Pxz*inv(Pvnew)=Ppre*H'*inv(H*Ppre*H'+R)
%滤波值xest：
%xest(:,k)=xpre(:,k)+K*vnew(:,k)=xpre(:,k)+K*(z(:,k)-H*xpre(:,k))
%滤波误差阵Pest：
%Pest=Ppre-K*Pvnew*K'=(I-K*H)*Ppre
%
K=Pxzk/Pvnewk;
xestk=xprek+K*vnewk;
Pestk=Pprek-K*Pvnewk*K';
Pestk=0.5*(Pestk+Pestk');