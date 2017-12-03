% Harris Adam Charron
% Project from ROB501 - Computer Vision for Robotics

close all
clear all
clc

%%
% Part 1: Image Smoothing and Subpixel Feature Extraction
% 	- The image was smoothed with a Gaussian filter
% 	- Corners were detected for each of the checkerboard images
% 	- Cross-junctions were verified at the predicted corners using a saddle point function based on the following literature:
% 		L. Lucchese and S. K. Mitra, “Using Saddle Points for Subpixel Feature Detection in Camera Calibration
% 		Targets,” in Proceedings of the Asia-Pacific Conference on Circuits and Systems (APCCAS),
% 		vol. 2, (Singapore), pp. 191–195, December 2002
% 	- Internal vs border corners were also identified - we were only looking for the internal corners between tiles of the checkerboard, not using its contour
% 	- Finally, cross-junction points were sorted in row-major order as per the checkerboard image

% Load all input checkerboard images
I = [];
I(:,:,1) = imread('../images/target_01.png');
I(:,:,2) = imread('../images/target_02.png');
I(:,:,3) = imread('../images/target_03.png');

% Load the bounding boxes surrounding each of the checkerboards
load('bounds.mat')
Ibounds = [];
Ibounds(:,:,1) = bounds.bpoly1;
Ibounds(:,:,2) = bounds.bpoly2;
Ibounds(:,:,3) = bounds.bpoly3;

% For each image, run the cross-junctions function on it to retrieve 
% ordered cross-junction points (corners)
Ipts = [];
for i = 1:size(I,3)
    Img = I(:,:,i);
    boundPoly = Ibounds(:,:,i);
    Ipts(:,:,i) = cross_junctions(Img, boundPoly);

    % Plot the initial image and the cross-junction points returned
    figure(i)
    imshow(uint8(Img));
    hold on
    plot(Ipts(1,:,i), Ipts(2,:,i), 'xy');
    hold on
end
hold off

%%
% Part 2: Camera Pose Estimation
% 	- Implement pose estimation using a nonlinear least squares procedure
% 	- Use the cross-junction points from Part 1

% World points for pose estimation
load('Wpts.mat')

% Build initial guess for camera 1 calibration H matrix
camera1 = load('camera_pose_01.mat');
initial_pose_guess_1 = build_H(camera1.camera.guess.C,camera1.camera.guess.t);
best_pose_1 = build_H(camera1.camera.best.C,camera1.camera.best.t);

% Build initial guess for camera 2 calibration H matrix
camera2 = load('camera_pose_02.mat');
initial_pose_guess_2 = build_H(camera2.camera.guess.C,camera2.camera.guess.t);
best_pose_2 = build_H(camera2.camera.best.C,camera2.camera.best.t);

% Build initial guess for camera 3 calibration H matrix
camera3 = load('camera_pose_03.mat');
initial_pose_guess_3 = build_H(camera3.camera.guess.C,camera3.camera.guess.t);
best_pose_3 = build_H(camera3.camera.best.C,camera3.camera.best.t);

% Put them all in array form for easy iteration
initial_pose_guesses = [];
initial_pose_guesses(:,:,1) = initial_pose_guess_1;
initial_pose_guesses(:,:,2) = initial_pose_guess_2;
initial_pose_guesses(:,:,3) = initial_pose_guess_3;
best_poses = [];
best_poses(:,:,1) = best_pose_1;
best_poses(:,:,2) = best_pose_2;
best_poses(:,:,3) = best_pose_3;

% Iterate through all images and determine the pose
error_mat = zeros(4,4,size(Ipts,3));
error = zeros(size(Ipts,3));
for i = 1:size(Ipts,3)
    E = pose_estimate_nlopt(initial_pose_guesses(:,:,i), Ipts(:,:,i), Wpts);
    error_mat(:,:,i) = abs(best_poses(:,:,i) - E);
    error(i) = sum(sum(error_mat(:,:,i)));
    fprintf('Error for camera %d: %f\n', i, error(i));
end

% Error for camera 1: 0.051208
% Error for camera 2: 0.111888
% Error for camera 3: 0.040949