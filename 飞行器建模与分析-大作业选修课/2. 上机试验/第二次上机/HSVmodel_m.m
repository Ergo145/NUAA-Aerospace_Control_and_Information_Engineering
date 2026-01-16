function [sys,x0,str,ts] = HSVmodel_m(t,x,u,flag) 

%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 11;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

d2r     =   pi/180;    %角度到弧度
%
% initialize the initial conditions
%
x0  = [6411000;118.8*d2r;32*d2r;3000;180*d2r;0*d2r;2*d2r;3*d2r;2*d2r;0*d2r;0*d2r;0*d2r];                   %设12个状态的初始值

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

m2ft    =   3.28084;   %米到英尺                      
X_mrc   =   124.01/m2ft;
g       =   9.65;                            

S_ref   =   334.73;          %参考面积             
c_ref   =   24.384;          %平均气动弦长              
b_ref   =   18.288;          %翼展长度                           
X_thrust=   60;              %主发动机推力中心到前缘距离                  
X_rc    =   7.5/m2ft/2;      %摆动发动机推力作用线与机体纵轴之间的距离

CD      =   u(1);
CL      =   u(2);
CC      =   u(3);   %以上三个为阻力，侧力和升力系数
Cll     =   u(4);
Cm      =   u(5);   
Cn      =   u(6);   %以上三个为气动力矩系数
Trx    =    u(7);   
Try    =    u(8);
Trz    =    u(9);    %以上三个为发动机产生的推力矢量在机体轴上的分解
M       =   u(10);   %飞行器瞬时质量               
q_bar   =   u(11);   %动压             
R       =   x(1);    %地心指向飞行器质心的位置矢量大小
tau     =   x(2);    %经度
delta   =   x(3);    %纬度
V       =   x(4);    %飞行速度              
kh      =   x(5);    %航迹方位角
gamma   =   x(6);    %航迹倾斜角
alpha   =   x(7);    %迎角
beta    =   x(8);    %侧滑角
sigma   =   x(9);    %航迹滚转角 
p       =   x(10);  
q       =   x(11);
r       =   x(12);   %以上三个为角速率  

Ixx     =   1.0e+004 *(-0.00000000709597*M^2+   0.00199093623971*M  -5.94301903021396) ;   
Iyy     =   1.0e+006 *(-0.00000000080331*M^2+   0.00021974209927*M  -1.69055469296301) ;    
Izz     =   Iyy;     %三个转动惯量　 (方程10到12使用)　

X_cg    =   0.00000000016469*M^2  -0.00005571500480*M+   7.36721170979675;  %质心到mrc的距离          
X_mass  =   X_mrc + X_cg;          %质心到前缘的距离                                                                                                  

D       =   q_bar*S_ref*CD;       %阻力  (方程4)
C       =   q_bar*S_ref*CC;       %侧力  (方程5和6)
L       =   q_bar*S_ref*CL;       %升力  (方程5和6)
m_mrc   =   q_bar*c_ref*S_ref*Cm; 
n_mrc   =   q_bar*b_ref*S_ref*Cn;  
Z       =   D*sin(alpha)+L*cos(alpha);

l_A     =   q_bar*b_ref*S_ref*Cll;           %滚转力矩   (方程10)
m_A     =   m_mrc   +  X_cg*Z;               %俯仰力矩   (方程11)  
n_A     =   n_mrc   +  X_cg*C*cos(beta);     %偏航力矩   (方程12) 
wcb     =   0.000073;                        %地球自转角速率 
%(方程1)
     dR         =   V*sin(gamma);
%(方程2)
     dtau = V*cos(gamma) * sin(kh) / ( R*cos(delta) ); 
%(方程3)     
     ddelta = V*cos(gamma) * cos(kh) / R;                                        %(方程3)
%(方程4)
     dV = -D/M - g*sin(gamma) + wcb^2*R*cos(delta)*( sin(gamma)*cos(delta) - cos(gamma)*sin(delta)*cos(kh) ); 
%(方程5)
     dkh = -( L*sin(sigma) - C*cos(sigma) )/( M*V*cos(gamma) ) + V*cos(gamma)*sin(kh)*tan(delta) / R + ...
        2*wcb*( sin(delta) - cos(delta)*tan(gamma)*cos(kh) ) +... 
        wcb^2*R*sin(delta)*cos(delta)*sin(kh) / ( V*cos(gamma) );                                         
%(方程6)
     dgamma = ( L*cos(sigma) + C*sin(sigma) ) / ( M*V ) + ( V/R - g/V )*cos(gamma)+ ...
        2*wcb*cos(delta)*sin(kh) + ...
        wcb^2*R*cos(delta)*( cos(delta)*cos(gamma) + sin(delta)*sin(gamma)*cos(kh) )/V;                                        
%(方程7)
     dalpha  =    q -tan(beta)*(p*cos(alpha)+r*sin(alpha))+ sin(sigma)/cos(beta)*...
                  (dkh*cos(gamma)- ddelta*sin(gamma)*sin(kh)+(dtau+wcb)*(cos(delta)*cos(kh)*sin(gamma)-sin(delta)*cos(gamma)))...
                   -cos(sigma)/cos(beta)*( dgamma-ddelta*cos(kh)-(dtau+wcb)*cos(delta)*sin(kh));    
%(方程7)  
      dbeta = p*sin(alpha) - r*cos(alpha) + sin(sigma)*( dgamma - ddelta*cos(kh) - ( dtau + wcb )*cos(delta)*sin(kh) ) + ...
        cos(sigma)*( dkh*cos(gamma) - ddelta*sin(kh)*sin(gamma) + ( dtau + wcb )*( cos(delta)*cos(kh)*sin(gamma) - sin(delta)*cos(gamma) ) );  
%(方程9)
      dsigma = -p*cos(alpha)*cos(beta) - q*sin(beta) - r*sin(alpha)*cos(beta) + dalpha*sin(beta)- ...
        dkh*sin(gamma) - ddelta*sin(kh)*cos(gamma) + (dtau + wcb)*( cos(delta)*cos(kh)*cos(gamma) + sin(delta)*sin(gamma) );
%(方程10) 
      dp = ( Iyy-Izz )*q*r/Ixx + l_A/Ixx;
%(方程11)
      dq = ( Izz-Ixx )*p*r/Iyy + m_A/Iyy;
%(方程12)
      dr = ( Ixx-Iyy )*p*q/Izz + n_A/Izz;                                     
%%%%%%%%%%%%%%%%%%%%------以上为再入飞行6自由度12状态方程（无动力）-------%%%%%%%%%%%%%%%%  

sys = [dR;dtau;ddelta;dV;dkh;dgamma;dalpha;dbeta;dsigma;dp;dq;dr];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];
% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
