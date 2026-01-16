function M = wgenmtr(r)
%生成反对称矩阵
M=[  0    -r(3)  r(2);
    r(3)   0    -r(1);
   -r(2)  r(1)    0 ];