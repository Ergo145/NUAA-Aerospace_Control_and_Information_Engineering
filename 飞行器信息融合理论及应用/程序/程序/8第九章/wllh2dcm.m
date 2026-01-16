function DCMen = wllh2dcm(lat,lon)


  if nargin<2,error('insufficient number of input arguments'),end

  clat = cos(pi/2-lat); slat = sin(pi/2-lat);
  clong = cos(pi/2+lon); slong = sin(pi/2+lon);
  
  C1 = [clong  slong 0; 
        -slong clong 0; 
           0     0   1];            % 先线绕z轴转90°+long
 
  C2 = [1   0     0 ;   
        0  clat slat;
        0 -slat clat];              % 再绕x轴转90°-lat

   DCMen = C2*C1;  % 地球坐标系到地理坐标系（ENU）
  