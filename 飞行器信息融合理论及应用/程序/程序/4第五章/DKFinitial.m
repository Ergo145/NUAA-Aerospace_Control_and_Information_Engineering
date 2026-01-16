function[Q,xestkdelay,Pestkdelay,N,Ppre,xpre,xr,Kgain,xest,Pest]=DKFinitial(Qd,x0,P0,SampleNo)
%
%状态方程：x[n+1]=Phi*x[n]+w[n]
%量测方程：y[n]=H*x[n]+v[n]
%
%Q为过程噪声方差阵，R为量测噪声方差阵
%Phi代表状态转移阵
%噪声方差设定 连续系统需先离散化
Q=Qd;
%初值设定
xestkdelay=x0;
Pestkdelay=P0;
%采样数设定
N=SampleNo;
%预设数据存储空间，以便调用数据记录函数
Ppre=[];xpre=[];xr=[];Kgain=[];xest=[];Pest=[];