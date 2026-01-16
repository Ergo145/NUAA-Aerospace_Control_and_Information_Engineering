function AirQuality=Sub_Height2AirQuality(Height) 

h = Height;          %几何高度，是地心到飞行器质心距离减去地球半径
R0=6356766.0;        %有效地球半径，不是实际半径，用来计算位势高度
g0=9.80665;          %标准海平面重力加速度
m0=0.0289644;
RsAtmosTempr=8.31432; %大气温度

Z=R0*h/(R0+h);                      
if (Z<174.9)
      disp('h can not be lower than 0.175km， because there is a departure point. Here, it is 175m high ');
      AtmosTemp=0;
      AtmosPress=0;
      AtmosDensity=0;
      SoundSpeed=0;
elseif (174.9<=Z)&(Z<11000)         
     AtmosTemp=288.15-0.0065*Z;
     AtmosPress=1.01325e+5*power(1-0.225577e-4*Z,5.25588);
     AtmosDensity=1.225*power(1-0.225577e-4*Z,4.25588);
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (11000<=Z)&(Z<20000)
     AtmosTemp=216.65;
     AtmosPress=2.263204e+4*exp(-1.576885e-4*(Z-11000));
     AtmosDensity=0.3639176*exp(-1.576885e-4*(Z-11000));
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (20000<=Z)&(Z<32000)
     AtmosTemp=216.65+0.001*(Z-20000);
     AtmosPress=5.474849e+3*power(1+4.615740e-6*(Z-20000),-34.16322);
     AtmosDensity=8.803471e-2*power(1+4.615740e-6*(Z-20000),-35.16322);
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (32000<=Z)&(Z<47000)
     AtmosTemp=228.65+0.0028*(Z-32000);
     AtmosPress=8.680160e+2*power(1+1.224579e-5*(Z-32000),-12.20115);
     AtmosDensity=1.322497e-2*power(1+1.224579e-5*(Z-32000),-13.20115);
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (47000<=Z)&(Z<51000)
     AtmosTemp=270.65;
     AtmosPress=1.109058e+2*exp(-1.262266e-4*(Z-47000));
     AtmosDensity=1.427527e-3*exp(-1.262266e-4*(Z-47000));
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (51000<=Z)&(Z<71000)
     AtmosTemp=270.65-0.0028*(Z-51000);
     AtmosPress=66.93853*power(1-1.034546e-5*(Z-51000),12.20115);
     AtmosDensity=8.616011e-4*power(1-1.034546e-5*(Z-51000),11.20115);
     SoundSpeed=20.0468*sqrt(AtmosTemp);
elseif (71000<=Z)&(Z<84852)
     AtmosTemp=214.65-0.002*(Z-71000);
     AtmosPress=3.956392*power(1-9.317494e-6*(Z-71000),17.08161);
     AtmosDensity=6.421057e-5*power(1-9.317494e-6*(Z-71000),16.08161);
     SoundSpeed=20.0468*sqrt(AtmosTemp);
else
    if (84852<=h)&(h<91000)            
        AtmosTemp=186.87;
        AtmosPress=1.01325e-1*(2.2730+1.042e-6*h)*exp((87284.8-h)/5470);
        AtmosDensity=4.4603475e-6*exp((87284.8-h)/5470);
        SoundSpeed=275.7302;
    elseif (91000<=h)&(h<110000)
        AtmosTemp=263.1905-76.3232*sqrt(1-power((h-91000)/(-19942.9),2));
         AtmosPress=1.01325e-1*(2.2730+1.042e-6*h)*exp((87284.8-h)/5470);
        AtmosDensity=4.4603475e-6*exp((87284.8-h)/5470);
        SoundSpeed=275.7302;
    elseif (110000<=h)&(h<120000)
        AtmosTemp=240.0+0.012*(h-110000);
         AtmosPress=1.01325e-1*(2.2730+1.042e-6*h)*exp((87284.8-h)/5470);
        AtmosDensity=4.4603475e-6*exp((87284.8-h)/5470);
        SoundSpeed=275.7302;
    elseif (120000<=h)&(h<1000000)
        AtmosTemp=1000-640.0*exp(-0.000018758*(h-120000)*(R0+120000)/(R0+h));
        AtmosPress=1.01325e-1*(2.2730+1.042e-6*h)*exp((87284.8-h)/5470);
        AtmosDensity=4.4603475e-6*exp((87284.8-h)/5470);
        SoundSpeed=275.7302;
    else
      error('h can not be higher than 1000km');
    end
end
AirQuality=[AtmosTemp;SoundSpeed;AtmosPress;AtmosDensity];