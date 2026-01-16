global d2r      m2ft        Kg2slug     Kg2lb       g 
global S_ref    c_ref       b_ref       X_mrc       X_thrust    X_rc
global MAX_RE   MIN_RE      MAX_LE      MIN_LE      MAX_RUD     MIN_RUD

d2r     =   pi/180;          %角度到弧度                      
m2ft    =   3.28084;         %米到英尺                      
Kg2slug =   0.0685218;       %公斤到斯勒格(质量单位=32.2磅)                     
Kg2lb   =   2.20462;         %公斤到磅                    
g       =   9.65;                            

S_ref   =   334.73;          %参考面积             
c_ref   =   24.384;          %平均气动弦长              
b_ref   =   18.288;          %翼展长度              
X_mrc   =   124.01/m2ft;     %mrc到前缘距离=37.7952英尺              
X_thrust=   60;              %主发动机推力中心到前缘距离                  
X_rc    =   7.5/m2ft/2;      %发动机推力作用线与机体纵轴之间的距离               
MAX_RE = 30;                                
MIN_RE = -  30;                            
MAX_LE = 30;                               
MIN_LE = -  30;                            
MAX_RUD = 30;                              
MIN_RUD = - 30;                            



