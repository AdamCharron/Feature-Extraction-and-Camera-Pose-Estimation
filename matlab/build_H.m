function H = build_H(C,t)
% Helper function to build an H matrix of the form:
%   H = [C|t;zeros(1,3),1]
% For pose information

H = eye(4);
H(1:3,1:3) = C;
H(1:3,4) = t;
end