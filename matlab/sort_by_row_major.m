function sorted_pts = sort_by_row_major(pts, bnds)
    % Takes in a list of points
    % Figures out where they are on the checkerboard, and sorts them in
    % row-major order
    % Returns an array of checkerboard dimensions, with values that are 
    % their corresponding indices in the pts list
    
    % Use homography to calculate the ratio between the distances to the
    % horizontal and vertical bounds of the image
    % First, convert to homog form
    polyBound1 = [bnds(:,1);1];
    polyBound2 = [bnds(:,2);1];
    polyBound3 = [bnds(:,3);1];
    polyBound4 = [bnds(:,4);1];
    
    % Second, use the cross product to compute the bounding lines between
    % the points in homog form
    T = cross(polyBound1, polyBound2)/norm(cross(polyBound1, polyBound2));
    R = cross(polyBound2, polyBound3)/norm(cross(polyBound2, polyBound3));
    B = cross(polyBound3, polyBound4)/norm(cross(polyBound3, polyBound4));
    L = cross(polyBound4, polyBound1)/norm(cross(polyBound4, polyBound1));
    
    % Ratio array will be a second array whose 1st row is ratios between
    % left and right line distances from each point. 2nd row will be the
    % same thing but vertical (top and bottom)
    % Get these distances in homog form by taking dot product of line and
    % point
    ratio_array = zeros(size(pts));
    for i=1:size(pts,2)
       temp_pt = [pts(:,i);1];
       ratio_array(1,i) = dot(L,temp_pt)/(dot(R,temp_pt) + dot(R,temp_pt));
       ratio_array(2,i) = dot(T,temp_pt)/(dot(T,temp_pt) + dot(B,temp_pt)); 
    end
    
    % Sort the pts array by
    %   i) Appending the pts and their homog distance ratios into 1 array
    %   ii) Sorting that array by rows based on vertical homog ratios
    %   iii) Sorting that array by rows based on hoizontal homog ratios
    sorted_pts = [];
    sortable = [pts; ratio_array];
    final_array = zeros(size(sortable));
    row_sort = sortrows(sortable',4)';
    for i = 1:8:size(row_sort,2)
        subtable = row_sort(:,i:i+7);
        final_array(:,i:i+7) = sortrows(subtable',3)';
        sorted_pts = cat(2,sorted_pts,final_array(1:2,i:i+7));
    end
end