function [pt] = saddle_point(I)
% SADDLE_POINT Locate saddle point in an image patch.
%
%   [pt] = SADDLE_POINT(I) finds the subpixel center of a cross-junction in the 
%   image patch I, by blurring the patch, fitting a hyperbolic paraboloid to 
%   it, and then finding the critical point of that paraboloid.
%
%   Note that the location of 'p' is relative to (0.5, 0.5) at the upper left 
%   corner of the patch, i.e., the pixels are treated as covering an area of
%   one unit square.
%
%   Inputs:
%   -------
%    I  - mxn image patch (grayscale, double or integer class).
%
%   Outputs:
%   --------
%    pt  - 2x1 subpixel location of saddle point in I (x, y coords).
%
% References:
%
%   L. Lucchese and S. K. Mitra, "Using Saddle Points for Subpixel Feature
%   Detection in Camera Calibration Targets," in Proc. Asia-Pacific Conf.
%   Circuits and Systems (APCCAS'02), vol. 2, (Singapore), pp. 191-195,
%   Dec. 2002.

sigma = 3;    % Must be between 1 and 3
wndSize = 13;  % Must be between 11 adn 21
min_size = min(size(I));
I = gaussian_blur(I, min(wndSize,min_size), sigma);

m = size(I,1);
n = size(I,2);
z = zeros(m*n,1);
X = zeros(m*n,6);

% Create X and z matrices: 
%   X is [1 x y xy x^2 y^2] for all x,y combos as a (mxn)x6 matrix
%   z is a (mxn)x1 matrix of pixel values 
for i = 1:m
   for j = 1:n
      tempX = [1, j, i, j*i, j^2, i^2];
      X = cat(1,X,double(tempX));
      z = cat(1,z,double(I(i,j)));
   end
end

% Get a least squares
a = pinv(X)*z;

% From calculations done from the Saddle Point paper, solve for where the 2
% lines intersect
x = (a(4)*a(3) - 2*a(6)*a(2))/(4*a(5)*a(6) - a(4)^2);
y = -a(2)/a(4) - 2*a(5)*x/a(4);
pt = [y;x];      
  
end