/*
 *
 *    Modified: Robert Relyea rer3378@rit.edu
 *
 * This code illustrates bird's eye view perspective transformation using opencv
 * Paper: Distance Determination for an Automobile Environment using Inverse Perspective Mapping in OpenCV  
 * Link to paper: https://www.researchgate.net/publication/224195999_Distance_determination_for_an_automobile_environment_using_Inverse_Perspective_Mapping_in_OpenCV
 * Code taken from: http://www.aizac.info/birds-eye-view-homography-using-opencv/
 *
 */ 

#include "ipm.hpp"

// Constructor
ipm::ipm()
{

  alpha =((double)alpha_val -90) * PI/180;
  beta =((double)beta_val -90) * PI/180;
  gamma =((double)gamma_val -90) * PI/180;
  focalLength = (double)f_val;
  dist = (double)dist_val;


  imageSize = cv::Size(cameraWidth, cameraHeight);

  w = cameraWidth;
  h = cameraHeight;

  previewImageSize = cv::Size(w, h);

  // Fisheye undistort
  // Camera Intrinsic Matrix
  cameraMatrix = (cv::Mat_<double>(3,3) <<
    1.3878727764994030e+03, 0,    cameraWidth/2,
    0,    1.7987055172413220e+03,   cameraHeight/2,
    0,    0,    1);

  // Distortion Coeffs //-3.3242373433775346e-01, 
  distCoeffs = (cv::Mat_<double>(4,1) <<
    -5.8881725390917083e-01, 5.8472404395779809e-01,
    -2.8299599929891900e-01, 0);

  // Create new camera matrix for fisheye
  cv::fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize,
                                                          cv::Matx33d::eye(), newCamMat, 1);
  // Create new maps for removing fisheye distortion
  cv::fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Matx33d::eye(), newCamMat, imageSize,
                                     CV_16SC2, map1, map2);


  // Perspective Mapping
  // Projection matrix 2D -> 3D
  A1 = (cv::Mat_<float>(4, 3)<< 
    1, 0, -w/2,
    0, 1, -h/2,
    0, 0, 0,
    0, 0, 1 );

  // Rotation matrices Rx, Ry, Rz
  RX = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, 0,
    0, cos(alpha), -sin(alpha), 0,
    0, sin(alpha), cos(alpha), 0,
    0, 0, 0, 1 );

  RY = (cv::Mat_<float>(4, 4) << 
    cos(beta), 0, -sin(beta), 0,
    0, 1, 0, 0,
    sin(beta), 0, cos(beta), 0,
    0, 0, 0, 1  );

  RZ = (cv::Mat_<float>(4, 4) << 
    cos(gamma), -sin(gamma), 0, 0,
    sin(gamma), cos(gamma), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1  );


  // R - rotation matrix
  R = RX * RY * RZ;

  // T - translation matrix
  T = (cv::Mat_<float>(4, 4) << 
    1, 0, 0, 0,  
    0, 1, 0, 0,  
    0, 0, 1, dist,  
    0, 0, 0, 1); 

  // K - intrinsic matrix 
  K = (cv::Mat_<float>(3, 4) << 
    focalLength/2, 0, w/2, 0,
    0, focalLength/2, h/2, 0,
    0, 0, 1, 0
    ); 

  transformationMat = K * (T * (R * A1));
}

cv::Mat ipm::mapPerspective(cv::Mat source)
{
  cv::Mat destination;

  cv::warpPerspective(source, destination, transformationMat, previewImageSize, cv::INTER_CUBIC | cv::WARP_INVERSE_MAP);

  cv::resize(destination, destination,cv::Size(frameWidth, frameHeight));

  return destination;
}

cv::Mat ipm::removeDistortion(cv::Mat source)
{
  cv::Mat destination;

  cv::resize(source, source, cv::Size(cameraWidth, cameraHeight));
  cv::remap(source, destination, map1, map2, cv::INTER_LINEAR);

  return destination;
}
    