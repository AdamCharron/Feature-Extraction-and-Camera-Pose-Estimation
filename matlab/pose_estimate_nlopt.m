function [E] = pose_estimate_nlopt(Eg, Ipts, Wpts)
%  POSE_ESTIMATE_NLOPT Estimate camera pose from 2D-3D correspondences via NLS.
%
%   [E] = POSE_ESTIMATE_NLOPT(Eg, Ipts, Wpts) performs a nonlinear least squares 
%   optimization procedure to determine the best estimate of the camera pose in 
%   the calibration target frame, given 2D-3D point correspondences.
%
%   Inputs:
%   -------
%    Eg    - 4x4 homogenous pose matrix, initial guess for camera pose.
%    Ipts  - 2xn array of cross-junction points (with subpixel accuracy).
%    Wpts  - 3xn array of world points (one-to-one correspondence with image).
%
%   Outputs:
%   --------
%    E  - 4x4 homogenous pose matrix, estimate of camera pose in target frame.

% Camera Intrinsics matrix from project paper
K = [564.9 0 337.3; 0 564.3 226.5; 0 0 1];

% To avoid it running off forever, set a max iteration limit
break_iter = 150;

% B = [d', angles']
Bk = [Eg(1:3,4); rpy_from_dcm(Eg(1:3,1:3))];  % Bk

% General idea:
%   Start with an original estimate of where the camera may be (homog Eg)
%   We know where the cross junctions are, and where the real points are
%   Using gradient descent, will update estimate Homog function H
%   Do this using Jacobians, which are estimated by find_jacobian.m

for iter = 1:break_iter
    
    % Build estimated H
    R_est = dcm_from_rpy(Bk(4:6));
    H_est = eye(4); 
    H_est(1:3,1:3) = R_est;
    H_est(1:3,4) = Bk(1:3);  
    
    % Sum up values to get estimate A and b matrices over all points
    % From them, get position offset estimate
    A = zeros(6);
    b = zeros(6,1);
    for i = 1:size(Ipts,2)
        J = find_jacobian(K, H_est, Wpts(:,i));
        A = A + J'*J;
        temp = K*(R_est\(Wpts(:,i) - H_est(1:3,4)));
        b = b + J'*(Ipts(:,i) - [1 0 0;0 1 0]*temp/temp(3));
    end
    
    % Sum update of estimate
    Bk = Bk + A\b;
end

% Build final estimate for H matrix
E = eye(4); 
E(1:3,1:3) = dcm_from_rpy(Bk(4:6));
E(1:3,4) = Bk(1:3);

end
