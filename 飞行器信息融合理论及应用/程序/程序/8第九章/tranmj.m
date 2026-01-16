function  CF = tranmj(wen,wie,fib,Rm,Rn,lat,v,h)%fib-fN
    ve=v(1);vn=v(2);vu=v(3);
    CF11 = [       0            wie(3)+wen(3)    -(wie(2)+wen(2));
             -(wie(3)+wen(3))         0             wie(1)+wen(1);  
              wie(2)+wen(2)    -(wie(1)+wen(1))        0         ];
    CF12 = [        0   -1/(Rm+h)   0;
                1/(Rn+h)    0       0;
            tan(lat)/(Rn+h) 0       0];
    CF13 = [        0                       0       vn/((Rm+h)^2);
                -wie(3)                     0       -ve/((Rn+h)^2);
            wie(2)+ve/((Rn+h)*cos(lat)^2)   0   -ve*tan(lat)/((Rn+h)^2)];
    CF21 =[       0     -fib(3)     fib(2);
             fib(3)       0          -fib(1);
            -fib(2)     fib(1)         0   ];
    CF22 = [(vn*tan(lat)-vu)/(Rn+h)   2*wie(3)+wen(3)   -(2*wie(2)+wen(2));
             -(2*wie(3)+wen(3))         -vu/(Rm+h)         wen(1);
              2*wie(2)+wen(2)            -2*wen(1)            0];
    CF23 = [2*(wie(3)*vu+wie(2)*vn)+ve*vn/((Rn+h)*cos(lat)^2)  0  (ve*vu-ve*vn*tan(lat))/((Rn+h)^2);
              -(ve^2/((Rn+h)*cos(lat)^2)+2*ve*wie(2))          0  vn*vu/((Rm+h)^2)+ve^2*tan(lat)/((Rn+h)^2);
                              -2*ve*wie(3)                     0  -(vn^2/((Rm+h)^2)+ve^2/((Rn+h)^2))];
    CF31 = zeros(3,3);
    CF32 = [    0                   1/(Rm+h) 0;
            1/((Rn+h)*cos(lat))         0    0;
                0                       0    1];
    CF33 = [        0                     0     -vn/((Rm+h)^2);
            ve*tan(lat)/((Rn+h)*cos(lat)) 0 -ve/((Rn+h)^2*cos(lat));
                    0                     0         0              ];
    CF = [CF11 CF12 CF13;CF21 CF22 CF23;CF31 CF32 CF33];
 
    
    
    
    
    
    
    