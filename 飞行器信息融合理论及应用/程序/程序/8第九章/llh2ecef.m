function Re_eb = llh2ecef(llh_t,rp,Cep)
f = 1.0/298.257;  %椭圆度
lon = llh_t(1);
lat = llh_t(2); 
h = llh_t(3);
x = (rp+h)*cos(lat)*cos(lon);
y = (rp+h)*cos(lat)*sin(lon);
z = (rp*(1-f)^2+h)*sin(lat);  %地球坐标系中的位置
 
xd=Cep(1,1)*x+Cep(1,2)*y+Cep(1,3)*z;
yd=Cep(2,1)*x+Cep(2,2)*y+Cep(2,3)*z;
zd=Cep(3,1)*x+Cep(3,2)*y+Cep(3,3)*z;  %以ecef原点为原点的切平面中的坐标系

Re_eb = [x,y,z,xd,yd,zd];

 

