function max_mat = kernelMax(I, wndSize)
% Check all values in a kernel
% Return the largest one
    max_mat = zeros(size(I));
    wndMid = floor(wndSize/2);
    offset = wndSize - wndMid - 1;
    for i=1:size(I,1)
        for j=1:size(I,2)
            window = I(max(i - offset, 1):min(i + wndMid, size(I,1)), max(j - offset, 1):min(j + wndMid, size(I,2)));
            max_mat(i, j) = max(max(window));
        end
    end
end