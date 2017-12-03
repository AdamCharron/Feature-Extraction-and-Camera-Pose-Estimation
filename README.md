# Feature-Extraction-and-Camera-Pose-Estimation
Perform feature extraction on a checkerboard, and use this to estimate camera pose. Written in MATLAB.
Feature Extraction and Camera Pose Estimation for a project in a Computer Vision for Robotics course

The purpose of this project was to accurately estimate the pose of a camera relative to a known object, in this case a planar checkerboard used as a camera calibration target. This was done by extracting a set of known feature points from an image of the target. Correspondences between the observed 2D image coordinates and the known 3D coordinates
of the feature points then allowed the pose to be determined.
Image processing libraries were not allowed for this project

The project consisted of 2 parts:

Part 1: Image Smoothing and Subpixel Feature Extraction
	- The image was smoothed with a Gaussian filter
	- Corners were detected for each of the checkerboard images
	- Cross-junctions were verified at the predicted corners using a saddle point function based on the following literature:
		L. Lucchese and S. K. Mitra, “Using Saddle Points for Subpixel Feature Detection in Camera Calibration
		Targets,” in Proceedings of the Asia-Pacific Conference on Circuits and Systems (APCCAS),
		vol. 2, (Singapore), pp. 191–195, December 2002
	- Internal vs border corners were also identified - we were only looking for the internal corners between tiles of the checkerboard, not using its contour
	- Finally, cross-junction points were sorted in row-major order as per the checkerboard image

Part 2: Camera Pose Estimation
	- Implement pose estimation using a nonlinear least squares procedure
	- Use the cross-junction points from Part 1

matlab/tester.m is the main file. Parts 1 and 2 can be run from there
