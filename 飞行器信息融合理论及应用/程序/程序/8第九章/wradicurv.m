function [Rm,Rn] = wradicurv(lat)
%RADICURV		Calculate meridian and prime radii of curvature. 
%       
%	[Rm,Rp] = radicurv(lat)
%
%   INPUTS
%       lat = geodetic latitude in radians
%
%   OUTPUTS
%       Rm = meridian radius of curvature in meters
%       Rp = prime radius of curvature in meters
%
%   NOTE
%       In some literature Rp is also referred to as the 'normal'
%       or 'transverse' radius of curvature
%
%   REFERENCES
%       Brockstein, A. and J. Kouba, "Derivation of Free Inertial, General
%       Wander Azimuth Mechanization Equations," Litton Systems, Inc., 
%       Guidance and Control Systems Division, Woodland Hills, California,
%       June 1969, Revised June 1981.
%
%       Kayton, M. and W. Fried, AVIONICS NAVIGATION SYSTEMS, 2nd edition,
%       John Wiley & Sons, New York, 1997.
%
%       Titterton, D. and J. Weston, STRAPDOWN INERTIAL NAVIGATION
%       TECHNOLOGY, Peter Peregrinus Ltd. on behalf of the Institution
%       of Electrical Engineers, London, 1997.
%
%	M. & S. Braasch 3-98
%	Copyright (c) 1997-98 by GPSoft LLC
%	All Rights Reserved.
%
%% 根据纬度计算子午圈、卯酉圈曲率半径

if nargin<1,error('insufficient number of input arguments'),end

Re = 6378137.0;    % equatorial radius of the earth; WGS-84
e = 0.0818191908426;  % eccentricity of the earth ellipsoid
%根据地理纬度计算子午毛酉圈曲率半径，参考文献1，式（2.65），（2.66）
den = 1-e^2*(sin(lat))^2;
Rm = (Re*(1-e^2))/( (den)^(3/2) );
Rn = Re/( sqrt(den) );

