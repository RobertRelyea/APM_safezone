/*
 * Created on: Feb 5, 2018
 *     Author: Robert Relyea rer3378@rit.edu
 *
 * This code illustrates bird's eye view perspective transformation using opencv
 * Paper: Distance Determination for an Automobile Environment using Inverse Perspective Mapping in OpenCV  
 * Link to paper: https://www.researchgate.net/publication/224195999_Distance_determination_for_an_automobile_environment_using_Inverse_Perspective_Mapping_in_OpenCV
 * Code taken from: http://www.aizac.info/birds-eye-view-homography-using-opencv/
 *
 */ 

// OpenCV imports
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 	3.1415926

typedef enum ipm_params
{
	// Extrinsic params
	alpha_val 		= 22,
	beta_val 		= 87,
	gamma_val 		= 90,
	f_val		 	= 741,
	dist_val 		= 477,

	// Intrinsic params
	frameWidth 		= 600,
	frameHeight 	= 360,
	cameraWidth 	= 1920,
	cameraHeight 	= 1080,
}ipm_params;

class ipm
{
public: 
	// Constructor
	ipm();

	// Apply camera intrinsic matrix to remove lens distortion in source
	cv::Mat removeDistortion(cv::Mat source);

	// Apply inverse perspective mapping to source with extrinsic camera matrix
	cv::Mat mapPerspective(cv::Mat source);

private:
	double alpha, beta, gamma, focalLength, dist;

    double w;
    double h;
    cv::Size previewImageSize;
    cv::Size imageSize;

    // Distortion Matricies
    cv::Mat cameraMatrix, distCoeffs, map1, map2, newCamMat;

    // Projection Mapping Matricies
    cv::Mat A1, RX, RY, RZ, R, T, K, transformationMat; 
};