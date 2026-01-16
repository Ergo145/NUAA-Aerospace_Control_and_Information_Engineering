function [xestkdelay,Pestkdelay,xrkdelay]=Kdelaya(xestk,Pestk,xrk)
%
%延迟环节，将本次循环的估计值延迟给下一环节，进而进行预测，此为按理论模型生成量测时的延迟函数
%
xestkdelay=xestk;
Pestkdelay=Pestk;
xrkdelay=xrk;