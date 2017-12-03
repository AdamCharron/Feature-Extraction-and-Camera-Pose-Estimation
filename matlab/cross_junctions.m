function [Ipts] = cross_junctions(I, boundPoly)
% CROSS_JUNCTIONS Find cross-junctions in image with subpixel accuracy.
%
%   [Ipts] = CROSS_JUNCTION(I, boundPoly) locates a series of cross- 
%   junction points on a planar calibration target, where the target is
%   bounded in the image by the specified 4-sided polygon. The number of
%   cross-junctions identified should be equal to the number of world points.
%
%   Note also that the world and image points must be in *correspondence*,
%   that is, the first world point should map to the first image point, etc.
%
%   Inputs:
%   -------
%    I          - Image (grayscale, double or integer class).
%    boundPoly  - 2x4 array defining bounding polygon for target (clockwise).
%
%   Outputs:
%   --------
%    Ipts  - 2xn array of cross-junctions (x, y), relative to the upper left
%            corner of I.

% Detect corners with Harris detector implementation
[rows, cols] = harris_corners(I);

% Filter out points that are outside the bounds of the checkerboard
good_rows = [];
good_cols = [];
for i = 1:size(cols,1)
    if inpolygon(cols(i),rows(i),boundPoly(1,:),boundPoly(2,:))
        good_rows = cat(1, good_rows, rows(i));
        good_cols = cat(1, good_cols, cols(i));
    end
end

first_Ipts = [];
saddle_ratios = [];
for i = 1:size(good_rows)
   window = 11;
   window_side = floor(window/2);
   
   % Make sure that the window remains within the bounds of the image
   % Assign an offset whenever there is risk of it leaving the image
   R_centroid_offset = 0;
   C_centroid_offset = 0;
   if (good_cols(i)-window_side) < 1
       C_centroid_offset = 1-(good_cols(i)-window_side);
   elseif (good_cols(i)+window_side) > size(I,2)
       C_centroid_offset = size(I,1)-(good_cols(i)+window_side);
   end
   if (good_rows(i)-window_side) < 1
       R_centroid_offset = 1-(good_rows(i)-window_side);
   elseif (good_rows(i)+window_side) > size(I,1)
       R_centroid_offset = size(I,2)-(good_rows(i)+window_side);
   end
   
   % Create window that will be passed along to saddle point function
   x_bound = good_cols(i)-window_side+C_centroid_offset;
   y_bound = good_rows(i)-window_side+R_centroid_offset;
   subI = I(y_bound:y_bound+2*window_side, x_bound:x_bound+2*window_side);
   saddle_ratio = X_or_L(I, good_cols(i), good_rows(i));
   pt = saddle_point(subI);

   % Saddle function returns NaN,NAN if it is not a valid saddle point
   % This check makes sure that only valid saddle points are retained
   if ~isnan(pt)
       pt = [good_cols(i)-window_side+C_centroid_offset + round(pt(1)); good_rows(i)-window_side+R_centroid_offset + round(pt(2))];
       first_Ipts = cat(2,first_Ipts,pt);
       saddle_ratios = cat(2,saddle_ratios,saddle_ratio);
   end
end

% Sort saddle types by max, and get an array of the corresponding indices
[saddle_vals,saddle_indices] = sort(saddle_ratios, 'descend');

% One final check that all points are within the bounds of the checkerboard
% Also checking whether they are X or L saddle points based on the max M/2N
% Finally, once points have been determined to be valid, "duplicate" points
% (those whose distance are within a certain threshold of another valid
% point) will be removed
unsorted_Ipts = [];
delete_pairs = [];
for i = 1:size(saddle_indices,2)
   if size(unsorted_Ipts,2) == 48
       break
   end
   j = saddle_indices(i);
   if inpolygon(first_Ipts(1,j),first_Ipts(2,j),boundPoly(1,:),boundPoly(2,:))
       d_flag = 1;
       threshold_d = 10;
       for k = 1:size(unsorted_Ipts,2)
           if j ~= k
                d = real(sqrt((first_Ipts(1,j)-unsorted_Ipts(1,k))^2 + (first_Ipts(2,j)-unsorted_Ipts(2,k))^2));
                if d < threshold_d
                    already_found_flag = 0;
                    for m = 1:size(delete_pairs,2)
                       if isequal(delete_pairs(:,m),[j;k]) || isequal(delete_pairs(:,m),[k;j])
                           already_found_flag = 1;
                           break
                       end
                    end
                    if already_found_flag == 0
                        d_flag = 0;
                        delete_pairs = cat(2,delete_pairs,[j;k]);
                        break
                    end
                end
            end
       end
       if d_flag == 1
           unsorted_Ipts = cat(2, unsorted_Ipts, first_Ipts(:,j));
       end
   end
end

% Sort saddle points by row-major order
Ipts = sort_by_row_major(unsorted_Ipts, boundPoly);
  
end
