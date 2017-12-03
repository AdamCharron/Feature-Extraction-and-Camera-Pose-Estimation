function [Ib] = gaussian_blur(I, wndSize, sigma)
% GAUSSIAN_BLUR Smooth image with symmetric Gaussian filter.
%
%   [Ib] = GAUSSIAN_BLUR(I, wndSize, sigma) produces a filtered image Ib 
%   from I using a square Gaussian kernel with window size wndSize.
%
%   Inputs:
%   -------
%    I        - mxn intensity image.
%    wndSize  - Kernel window size (square, odd number of pixels).
%    sigma    - Standard deviation of Gaussian (pixels, symmetric).
%
%   Outputs:
%   --------
%    Ib  - mxn filtered output image, of same size and class as I.

% Create meshgrids that evenly spread values away from origin
[map1, map2] = meshgrid(-(wndSize-1)/2:(wndSize-1)/2, -(wndSize-1)/2:(wndSize-1)/2);

% Pass these values through gaussian function for each point on this grid
% This grid will act as a weight map that convolution will apply to each
% pixel
gauss = exp(-(map1.^2+map2.^2)/(2*sigma^2));

% Normalize the kernel
sum = 0;
[a,b] = size(gauss);
for i=1:a
   for j=1:b
        sum = sum + gauss(i,j);
   end
end
kernel = gauss./sum;

% Run convolution of the kernel on the image
Ib = conv2(I,kernel,'same');

% Handle types
if isa(I,'double')
    Ib = double(Ib); 
elseif isa(I,'uint8')
    Ib = uint8(Ib); 
end
  
end
