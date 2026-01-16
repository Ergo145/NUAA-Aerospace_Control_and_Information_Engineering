function DCMnb = wqua2dcm(qua_vec)
%QUA2DCM       Quaternion to direction cosine matrix conversion.
%       
%	 DCMnb = qua2dcm(qua_vec)
%
%   INPUT
%       qua_vec = 4 element quaternion vector
%               = [a b c d]
%       where: a = cos(MU/2)
%              b = (MUx/MU)*sin(MU/2)
%              c = (MUy/MU)*sin(MU/2)
%              d = (MUz/MU)*sin(MU/2)
%       where: MUx, MUy, MUz are the components of the angle vector
%              MU is the magnitude of the angle vector
%
%   OUTPUT
%       DCMnb = 3x3 direction cosine matrix providing the
%             transformation from the body frame
%             to the navigation frame
%

%   REFERENCE:  Titterton, D. and J. Weston, STRAPDOWN
%               INERTIAL NAVIGATION TECHNOLOGY, Peter
%               Peregrinus Ltd. on behalf of the Institution
%               of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 12-97
%	Copyright (c) 1997 by GPSoft
%	All Rights Reserved.
%
%% 四元数向DCM转换

  if nargin<1,error('insufficient number of input arguments'),end
  
  a = qua_vec(1); b = qua_vec(2); c = qua_vec(3); d = qua_vec(4);
  
  DCMnb(1,1) = a*a + b*b - c*c - d*d;
  DCMnb(1,2) = 2*(b*c + a*d);
  DCMnb(1,3) = 2*(b*d - a*c);
  DCMnb(2,1) = 2*(b*c - a*d);
  DCMnb(2,2) = a*a - b*b + c*c - d*d;
  DCMnb(2,3) = 2*(c*d + a*b);
  DCMnb(3,1) = 2*(b*d + a*c);
  DCMnb(3,2) = 2*(c*d - a*b);
  DCMnb(3,3) = a*a - b*b - c*c + d*d;
  