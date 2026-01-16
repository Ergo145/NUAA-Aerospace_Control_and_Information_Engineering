function [Ppre,xpre,xr,Kgain,xest,Pest]=recorda(Ppre,xpre,xr,Kgain,xest,Pest,Pprek,xprek,K,xestk,Pestk,xrk)
%
%该函数用于提取后面作图及分析所需的参量，此函数适用于按理论模型生成量测时
%使用前应先开辟存储空间，如设为空数组
%
Ppre=[Ppre diag(Pprek)];
xpre=[xpre xprek];
xr=[xr xrk];
Kgain=[Kgain K];
xest=[xest xestk];
Pest=[Pest diag(Pestk)];