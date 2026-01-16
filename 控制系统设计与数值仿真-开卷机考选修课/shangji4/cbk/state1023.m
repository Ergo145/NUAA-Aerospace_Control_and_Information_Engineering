function statedot = state1023(state,u,t,f)
global m omega

A =[0 ,1,0,0,0;
    0,0,1,0,0;
    0,-omega^2,0,1,0;
    0,0,0,0,1;
    0,0,0,0,0
    ];
B = [0,0,0,0,1/m]';

statedot = A*state+B*(u+f);

end