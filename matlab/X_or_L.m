function ratio = X_or_L(I, x, y)
  
    % Window to search over to determine junction type
    wndSize = 31;
    window_side = floor(wndSize/2);
   
    % Make sure that the window remains within the bounds of the image
    % Assign an offset whenever there is risk of it leaving the image
    R_centroid_offset = 0;
    C_centroid_offset = 0;
    if (x-window_side) < 1
        C_centroid_offset = 1-(x-window_side);
    elseif (x+window_side) > size(I,2)
        C_centroid_offset = size(I,2)-(x+window_side);
    end
    if (y-window_side) < 1
        R_centroid_offset = 1-(y-window_side);
    elseif (y+window_side) > size(I,1)
        R_centroid_offset = size(I,1)-(y+window_side);
    end
   
    % Create window that will be passed along to saddle point function
    x_bound = x-window_side+C_centroid_offset;
    y_bound = y-window_side+R_centroid_offset;
    subI = I(y_bound:y_bound+2*window_side, x_bound:x_bound+2*window_side);

    % Sum up the values of each of the 4 diagonal quadrants
    mid = floor(size(subI,1)/2) + 1;
    NWsum = sum(sum(subI(1:mid, 1:mid)));
    SWsum = sum(sum(subI(1:mid, mid:size(subI,1))));
    NEsum = sum(sum(subI(mid:size(subI,1), 1:mid)));
    SEsum = sum(sum(subI(mid:size(subI,1), mid:size(subI,1))));
    
    % Use M>2N check to get a ratio M/N - 2 to compare closeness to X
    sum_array = [NWsum, SWsum, NEsum, SEsum];
    [temp,temp] = sort(sum_array);
    M = sum(sum_array(temp([end,end-1])));
    N = sum(sum_array(temp([1,2])));
    ratio = (M/N)-2;
    
end