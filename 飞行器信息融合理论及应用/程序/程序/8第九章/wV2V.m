function V_err = wV2V(fib_b,Wie_n,Wen_n,V,Cnb,g)
%速度更新
global deltat 
fib_n=Cnb'*fib_b';
dv=fib_n-wgenmtr(2*Wie_n+Wen_n)*V'+[0;0;-g];
V_err=V'+dv*deltat;
