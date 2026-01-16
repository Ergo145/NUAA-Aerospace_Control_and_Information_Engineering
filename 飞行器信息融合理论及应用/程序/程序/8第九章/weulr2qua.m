function qua_vec = weulr2qua(eul_vect)
%EULR2QUA       Euler angle vector to quaternion conversion.
%       
%	qua_vec = eulr2qua(eul_vect)
%
%   INPUTS
%       eul_vect(1) = pitch angle in radians 
%       eul_vect(2) = roll angle in radians 
%       eul_vect(3) = yaw angle in radians 
%
%   OUTPUTS
%       qua_vec = 4 element quaternion vector
%               = [a b c d]
%       where: a = cos(MU/2)
%              b = (MUx/MU)*sin(MU/2)
%              c = (MUy/MU)*sin(MU/2)
%              d = (MUz/MU)*sin(MU/2)
%       where: MUx, MUy, MUz are the components of the angle vector
%              MU is the magnitude of the angle vector
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
%% 从欧拉角转换到四元数

  if nargin<1,error('insufficient number of input arguments'),end
  %俯仰、横滚、航向
  theta = eul_vect(1); phi = eul_vect(2); psi = eul_vect(3);
%[theta gama fai]
  cthe2 = cos(theta/2); sthe2 = sin(theta/2);
  cphi2 = cos(phi/2); sphi2 = sin(phi/2);
  cpsi2 = cos(psi/2); spsi2 = sin(psi/2);

  a = cpsi2*cthe2*cphi2 - spsi2*sthe2*sphi2;
  b = cpsi2*sthe2*cphi2 - spsi2*cthe2*sphi2;
  c = cpsi2*cthe2*sphi2 + spsi2*sthe2*cphi2;
  d = cpsi2*sthe2*sphi2 + spsi2*cthe2*cphi2;
%   Q=[cos(fai/2)*cos(theta/2)*cos(gama/2)-sin(fai/2)*sin(theta/2)*sin(gama/2);
%    cos(fai/2)*sin(theta/2)*cos(gama/2)-sin(fai/2)*cos(theta/2)*sin(gama/2);
%    cos(fai/2)*cos(theta/2)*sin(gama/2)+sin(fai/2)*sin(theta/2)*cos(gama/2);
%    cos(fai/2)*sin(theta/2)*sin(gama/2)+sin(fai/2)*cos(theta/2)*cos(gama/2)];

  qua_vec = [a b c d]';
  