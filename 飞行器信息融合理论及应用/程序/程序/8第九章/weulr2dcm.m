function DCMnb = weulr2dcm(eul_vect)
%EULR2DCM       Euler angle vector to direction cosine
%               matrix conversion.
%       
%	DCMnb = eulr2dcm(eul_vect)
%
%   INPUTS
%       eul_vect(1) = roll angle in radians 
%
%       eul_vect(2) = pitch angle in radians 
%
%       eul_vect(3) = yaw angle in radians 
%
%   OUTPUTS
%       DCMnb = 3x3 direction cosine matrix providing the
%             transformation from the navigation frame
%             to the body frame
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
%   DCMnb=[      cthe*cpsi                   cthe*spsi              -sthe;
%          sthe*sphi*cpsi-spsi*cphi  sthe*sphi*spsi+cphi*cpsi   sphi*cthe;
%          cphi*cpsi*sthe+sphi*sthe  cphi*sthe*spsi-sphi*cyhe   cphi*cthe;]
%
%% Jr9910£º2013/05/06£¬Å·À­½ÇÏòCn_b×ª»»£¬×ª¶¯Ë³Ðò£ºÆ«º½->¸©Ñö->¹ö×ª

  if nargin<1,error('insufficient number of input arguments'),end

  theta = eul_vect(1); phi = eul_vect(2); psi = eul_vect(3);

  cpsi = cos(psi); spsi = sin(psi);
  cthe = cos(theta); sthe = sin(theta);
  cphi = cos(phi); sphi = sin(phi);

  C1 = [cpsi  spsi 0; ...
        -spsi cpsi 0; ...
         0     0   1];
  C2 = [cphi  0  -sphi; ...
          0   1     0 ; ...
        sphi  0   cphi];
  C3 = [1   0    0;   ...
        0  cthe sthe; ...
        0 -sthe cthe];  

  DCMnb = C2 * C3 * C1;
