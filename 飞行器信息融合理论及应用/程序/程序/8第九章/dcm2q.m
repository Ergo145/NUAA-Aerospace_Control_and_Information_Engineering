function q = dcm2q(Cnb,qr)

 q(1,1) = qr*0.5*sqrt(abs(1+Cnb(1,1)+Cnb(2,2)+Cnb(3,3)));
 q(2,1) = (Cnb(2,3)-Cnb(3,2))/(4*q(1,1));
 q(3,1) = (Cnb(3,1)-Cnb(1,3))/(4*q(1,1));
 q(4,1) = (Cnb(1,2)-Cnb(2,1))/(4*q(1,1));