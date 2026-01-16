function [out]=coefficients_new(in)                %较简单的再入气动参数,用来模拟实际飞行器
global d2r                                         %量纲转化
global c_ref       b_ref                           %平均气动弦长；翼展长度
global MAX_RE   MIN_RE      MAX_LE      MIN_LE     %舵偏角限制 
    
LE      =   in(1);  %左升降副翼舵偏角                              
RE      =   in(2);  %右升降副翼舵偏角                             
RUD     =   in(3);  %方向舵偏角                            
V       =   in(4);  %飞行速度                              
p       =   in(5);  %滚转角速率                             
q       =   in(6);  %俯仰角速率                             
r       =   in(7);  %偏航角速率                             
MACH    =   in(8);  %马赫数                             
alpha   =   in(9);  %迎角                              
beta    =   in(10); %侧滑角                              

ALP=     alpha/d2r;   %迎角转化为角度

CDa     = 8.717E-02 - 3.307E-02*MACH + 3.179E-03*ALP - 1.250E-04*(ALP*MACH) + 5.036E-03*MACH^2 -1.100E-03*ALP^2 + 1.405E-07*(ALP*MACH)^2 ...
	  - 3.658E-04*MACH^3 + 3.175E-04*ALP^3 + 1.274E-05*MACH^4 -2.985E-05*ALP^4 - 1.705E-07 *MACH^5 + 9.766E-07*ALP^5;   %基本阻力系数
CD_LE   =   0;
CD_RE   =   0;
CD_RUD  =   0;


%================================================
%                C_L 
%================================================
CLa	    =	- 8.19E-02 + 4.70E-02*MACH + 1.86E-02*ALP - 4.73E-04*(ALP*MACH) -9.19E-03*MACH^2 -1.52E-04*ALP^2 + 5.99E-07*(ALP*MACH)^2 ...
      + 7.74E-04*	MACH^3 + 4.08E-06 *ALP^3 - 2.93E-05*MACH^4 + -3.91E-07*	ALP^4 + 4.12E-07*MACH^5 + 1.30E-08*ALP^5; %基本升力系数
CL_LE   =   0;
CL_RE   =   0;

%================================================
%                C_m 
%================================================
Cma     =   - 2.192E-02 + 7.739E-03*MACH - 2.260E-03*ALP + 1.808E-04*(ALP*MACH) - 8.849E-04*MACH^2 + 2.616E-04*ALP^2 -2.880E-07*(ALP*MACH)^2 ...
		 + 4.617E-05*MACH^3 -7.887E-05*ALP^3 -1.143E-06*MACH^4 + 8.288E-06*ALP^4 + 1.082E-08*MACH^5 - 2.789E-07*ALP^5; %基本俯仰力矩系数
Cmq     =   - 1.36E+00 + 3.86E-01*MACH + 7.85E-04*ALP + 1.40E-04*(ALP*MACH)	- 5.42E-02 *MACH^2	+ 2.36E-03*ALP^2	-1.95E-06 *(ALP*MACH)^2	...
	       + 3.80E-03*MACH^3 - 1.48E-03 *ALP^3 - 1.30E-04 *MACH^4 + 1.69E-04*ALP^4	+ 1.71E-06*MACH^5	- 5.93E-06 *ALP^5; %q引起的俯仰力矩增量系数
Cm_LE   =   (0.0292/2)*d2r;
Cm_RE   =   (0.0292/2)*d2r;
Cm_RUD  =    0;

%================================================
%                C_Y 
%================================================
CYbv	=-1.76E+00 + 4.58E-01*MACH - 3.26E-03*ALP + 3.80E-05*(ALP*MACH) -6.36E-02*MACH^2 + 2.36E-03*ALP^2 + 3.45E-07*(ALP*MACH)^2 ...
	  + 4.44E-03* MACH^3 - 6.03E-04*ALP^3 -1.51E-04*MACH^4 + 4.52E-05*ALP^4 + 1.98E-06*MACH^5 -1.09E-06*ALP^5;  %基本侧力系数 
  
% CY_RE	
if(RE<=0)
    temp =  2.27E-07*MIN_RE + 4.11E-09 *(ALP*MACH)*MIN_RE  - 5.04E-08*MIN_RE^2 + 4.50E-14*((ALP*MACH)*MIN_RE)^2;
    CY_RE = temp/MIN_RE ;
 else
    temp = 2.27E-07*MAX_RE + 4.11E-09 *(ALP*MACH)*MAX_RE  - 5.04E-08*MAX_RE^2 + 4.50E-14*((ALP*MACH)*MAX_RE)^2;
    CY_RE = temp/MAX_RE;
end

% CY_LE	
if(LE<=0)
    temp =  -(2.27E-07*MIN_LE + 4.11E-09 *(ALP*MACH)*MIN_LE  - 5.04E-08*MIN_LE^2 + 4.50E-14*((ALP*MACH)*MIN_LE)^2);
    CY_LE = temp/MIN_LE;
 else
    temp = -(2.27E-07*MAX_LE + 4.11E-09 *(ALP*MACH)*MAX_LE  - 5.04E-08*MAX_LE^2 + 4.50E-14*((ALP*MACH)*MAX_LE)^2);
    CY_LE = temp/MAX_LE ;
 end

% CY_RUD 
CY_RUD =  3.84E-04 - 1.17E-05*(ALP) - 1.07E-05*(MACH) + 2.60E-07*(ALP*MACH);

% NEW CYb
CYb = CYbv;


%================================================
%                C_l 
%================================================
Cllbv	=- 1.402E-01 + 3.326E-02*MACH - 7.590E-04*ALP + 8.596E-06*(ALP*MACH) - 3.794E-03*MACH^2 + 2.354E-06*ALP^2 -1.044E-08*(ALP*MACH)^2 ...
	  + 2.219E-04*MACH^3 - 8.964E-18*ALP^3 - 6.462E-06*MACH^4 + 3.803E-19*ALP^4 + 7.419E-08*MACH^5 -3.353E-21*ALP^5;  

%  Cll_RE  
Cll_RE_AM = 3.570E-04 - 9.569E-05*ALP - 3.598E-05*MACH  + 4.950E-06*ALP^2 + 1.411E-06*MACH^2  -3.4800e-004 -1.3923e-008*(ALP*MACH)^2;
Cll_RE =  + 1.170E-04 + 2.794E-08*(ALP*MACH);

% Cll_LE 
Cll_LE_AM = -(3.570E-04 - 9.569E-05*ALP - 3.598E-05*MACH  + 4.950E-06*ALP^2 + 1.411E-06*MACH^2 -3.4800e-004 -1.3923e-008*(ALP*MACH)^2);
Cll_LE = -( +1.170E-04 + 2.794E-08*(ALP*MACH));

%   Cll_RUD 
Cll_RUD_AM =	- 5.0103E-19 + 6.2723E-20*ALP + 2.3418E-20*MACH  -3.4201E-21*(ALP*MACH); 
Cll_RUD =	 + 0.00011441 - 2.6824E-06*(ALP) - 3.5496E-06*(MACH)  + 5.5547E-08*(ALP*MACH);

%   Cllr 
Cllr = 3.82E-01 - 1.06E-01*MACH + 1.94E-03* ALP -8.15E-05*(ALP*MACH) + 1.45E-02*MACH^2 - 9.76E-06*ALP^2 + 4.49E-08*(ALP*MACH)^2 ...
	  - 1.02E-03*MACH^3 - 2.70E-07*ALP^3 + 3.56E-05*MACH^4 + 3.19E-08*ALP^4 - 4.81E-07*MACH^5 -1.06E-09*ALP^5;    %r引起的滚转力矩增量系数

Cllp = - 2.99E-01 + 7.47E-02*MACH + 1.38E-03*ALP - 8.78E-05*(ALP*MACH) - 9.13E-03*MACH^2 - 2.04E-04*ALP^2 - 1.52E-07*(ALP*MACH)^2	...
	  + 5.73E-04*MACH^3 - 3.86E-05*ALP^3 - 1.79E-05*MACH^4 + 4.21E-06*ALP^4 + 2.20E-07*MACH^5 - 1.15E-07*ALP^5;   %p引起的滚转力矩增量系数
  
% NEW Cllb
Cllb = Cllbv + Cll_RE_AM + Cll_LE_AM + Cll_RUD_AM;

%================================================
%                C_n 
%================================================
Cnbv =  3.68E-01 + 6.03E-16*(ALP) - 9.79E-02*(MACH)	- 3.84E-16*(ALP)^2 + 1.24E-02*(MACH)^2 + 8.58E-17*(ALP)^3	- 8.05E-04 *(MACH)^3 ...
	  - 7.75E-18 *(ALP)^4 + 2.57E-05*(MACH)^4 + 2.42E-19*(ALP)^5 - 3.20E-07 *(MACH)^5;   %基本偏航力矩系数

% Cn_RE	
if(RE<=0)  
     Cn_RE_AM = +2.10E-04 + 1.83E-05*ALP - 3.56E-05*MACH - 6.39E-07*ALP^2 + 8.16E-07*MACH^2;
     temp=-1.30E-05*MIN_RE - 8.93E-08*(ALP*MACH)*MIN_RE + 1.97E-06*MIN_RE^2+1.41E-11*((ALP*MACH)*MIN_RE)^2;
     Cn_RE = temp/MIN_RE;
 else
     Cn_RE_AM = +2.10E-04 + 1.83E-05*ALP - 3.56E-05*MACH - 6.39E-07*ALP^2 + 8.16E-07*MACH^2; 
     temp = -1.30E-05*MAX_RE - 8.93E-08*(ALP*MACH)*MAX_RE + 1.97E-06*MAX_RE^2+1.41E-11*((ALP*MACH)*MAX_RE)^2;
     Cn_RE = temp/MAX_RE;
 end
 
%Cn_LE	
if (LE<=0)  
     Cn_LE_AM = -(+2.10E-04 + 1.83E-05*ALP - 3.56E-05*MACH - 6.39E-07*ALP^2 + 8.16E-07*MACH^2);
     temp=-(-1.30E-05*MIN_LE - 8.93E-08*(ALP*MACH)*MIN_LE + 1.97E-06*MIN_LE^2+1.41E-11*((ALP*MACH)*MIN_LE)^2);
     Cn_LE = temp/MIN_LE ;
 else
     Cn_LE_AM = -(+2.10E-04 + 1.83E-05*ALP - 3.56E-05*MACH - 6.39E-07*ALP^2 + 8.16E-07*MACH^2); 
     temp = -(-1.30E-05*MAX_LE - 8.93E-08*(ALP*MACH)*MAX_LE + 1.97E-06*MAX_LE^2+1.41E-11*((ALP*MACH)*MAX_LE)^2);
     Cn_LE = temp/MAX_LE; 
 end

 % Cn_RUD 
 Cn_RUD_AM = 2.85E-18 - 3.59E-19*ALP -1.26E-19*MACH   + 1.57E-20*(ALP*MACH);
 Cn_RUD =  -5.56E-03 + 2.78E-05*(ALP)  + 3.30E-05*(MACH)  - 6.26E-07*(ALP*MACH);

 % NEW Cn_b
 Cnb= Cnbv + Cn_RE_AM + Cn_LE_AM + Cn_RUD_AM;
  
Cnp =  3.68E-01 -9.79E-02*MACH + 7.61E-16*ALP + 1.24E-02*MACH^2-4.64E-16*ALP^2 -8.05E-04*MACH^3 +1.01E-16*ALP^3 +2.57E-05*MACH^4+ ...
      -9.18E-18*ALP^4 -3.20E-07* MACH^5 + 2.96E-19*ALP^5;    %p引起的偏航力矩增量系数
Cnr =	- 2.41E+00 + 5.96E-01*MACH - 2.74E-03*ALP + 2.09E-04*(ALP*MACH) - 7.57E-02*MACH^2 + 1.15E-03*	ALP^2 - 6.53E-08*(ALP*MACH)^2 ...
	  + 4.90E-03*MACH^3 - 3.87E-04*ALP^3 - 1.57E-04*MACH^4 + 3.60E-05*ALP^4 + 1.96E-06*MACH^5 - 1.18E-06*ALP^5;  %r引起的偏航力矩增量系数   

CD  = CDa       + CD_RE*RE  + CD_LE*LE  + CD_RUD*RUD; 
CY  = CYb*beta  + CY_RE*RE  + CY_LE*LE  + CY_RUD*RUD;
CL  = CLa       + CL_RE*RE  + CL_LE*LE;
Cll = Cllb*beta + Cll_RE*RE + Cll_LE*LE + Cll_RUD*RUD   + Cllr*(r*b_ref)/(2*V) + Cllp*(p*b_ref)/(2*V);  
Cm  = Cma       + Cm_RE*RE  + Cm_LE*LE  + Cm_RUD*RUD    + Cmq*(q*c_ref)/(2*V);
Cn  = Cnb*beta  + Cn_RE*RE  + Cn_LE*LE  + Cn_RUD*RUD    + Cnp*(p*b_ref)/(2*V)   + Cnr*(r*b_ref)/(2*V);

coeff1  =   [CDa; CD_LE;  CD_RE; CD_RUD];  %阻力系数
coeff2  =   [CLa; CL_LE;  CL_RE];   %升力系数
coeff3  =   [CYb; CY_LE;  CY_RE;  CY_RUD]; %侧力系数
coeff4  =   [Cllb;Cll_LE; Cll_RE; Cll_RUD; Cllr; Cllp]; %滚转力矩系数
coeff5  =   [Cma; Cm_LE;  Cm_RE;  Cm_RUD;  Cmq];   %俯仰力矩系数
coeff6  =   [Cnb; Cn_LE;  Cn_RE;  Cn_RUD;  Cnr;   Cnp]; %偏航力矩系数
coeff   =   [coeff1;  coeff2;   coeff3; coeff4;  coeff5; coeff6];
 LD= CL/CD;

out = [CD;CL;CY;Cll;Cm;Cn;LD];   %阻力系数；升力系数；侧力系数；滚转力矩系数；俯仰力矩系数；偏航力矩系数

