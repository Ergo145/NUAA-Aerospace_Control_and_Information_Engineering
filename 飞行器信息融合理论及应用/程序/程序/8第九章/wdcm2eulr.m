function atti = wdcm2eulr(Cbn)
%Cbn从n到b
%DCM2EULR       Direction cosine matrix to Euler angle
%               vector conversion.
%       
%	eul_vect = dcm2eulr(DCMbn)
%
%   INPUTS
%       DCMbn = 3x3 direction cosine matrix providing the
%             transformation from the body frame
%             to the navigation frame
%
%   OUTPUTS
%       eul_vect(1) = roll angle in radians 
%
%       eul_vect(2) = pitch angle in radians 
%
%       eul_vect(3) = yaw angle in radians 
%
%   NOTE
%       If the pitch angle is vanishingly close to +/- pi/2,
%       the elements of EUL_VECT will be filled with NaN.

%   REFERENCE:  Titterton, D. and J. Weston, STRAPDOWN
%               INERTIAL NAVIGATION TECHNOLOGY, Peter
%               Peregrinus Ltd. on behalf of the Institution
%               of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 12-97
%	Copyright (c) 1997 by GPSoft
%	All Rights Reserved.
%
%定义
  if nargin < 1, error('insufficient number of input arguments'), end

 
  %姿态更新
  theta=asin(Cbn(2,3));
  gama=atan(-Cbn(1,3)/Cbn(3,3));
  fai=atan(-Cbn(2,1)/Cbn(2,2));

%主值判断  
 if Cbn(3,3)<0
        if Cbn(1,3)>0
            gama=gama-pi;
        else gama=gama+pi;
        end
 end
    
if Cbn(2,2)<0
    fai=fai+pi;
else
    if Cbn(2,1)>0
        fai=fai+2*pi;
    end
end
atti=[theta;gama;fai];

  
  
  
  
  
