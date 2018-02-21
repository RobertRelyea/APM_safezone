/*
 *
 *  Created on: Feb 5, 2018
 *      Author: Robert Relyea rer3378@rit.edu
 *
 */

// ROS Includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Safezone includes
#include "classifier.hpp"
#include "ipm.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;



int main(int argc, char** argv) {

  // ROS node initialization
  ros::init(argc, argv, "safezone");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<PointCloudT>("safezone_pc", 1);

  // ENet initialization
  string model_file   = "/home/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_model.prototxt";
  string trained_file = "/home/apm/apm_phase5/ENet/final_model_weights/bn_conv_merged_weights.caffemodel"; //for visualization

  Classifier classifier(model_file, trained_file);
  ipm mapper;

  string LUT_file = "/home/apm/apm_phase5/safezone.png";

  // Video capture initialization
  cv::VideoCapture cap("/home/apm/Videos/2017-12-01-142644.webm");
  // cv::VideoCapture cap("/dev/video0");

  cv::Mat img; // Create mat to hold new frames
  cv::Mat output_frame;

  while (ros::ok())
  {
    std::cout << "Grabbing new frame" << std::endl;
    cap >> img;  // Grab new frame

    output_frame = classifier.Predict(img, LUT_file);

    cv::resize(img, img, cv::Size(frameWidth, frameHeight));
    cv::imshow("Original Frame", img);
    cv::resize(output_frame, output_frame, cv::Size(frameWidth, frameHeight));
    cv::imshow("ENet Output Frame", output_frame);

    // Remove lens distortion
    output_frame = mapper.removeDistortion(output_frame);
    // Apply IPM
    output_frame = mapper.mapPerspective(output_frame);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->header.frame_id = "/my_frame";
    cloud->header.stamp = ros::Time::now().toNSec() / 1000;

    // Construct pointcloud from perspective mapped image

    cv::Mat cloud_frame;
    cv::resize(output_frame, cloud_frame, cv::Size(160, 90));

    for (int r = 0; r < 90; ++r)
    {
        for (int c = 0; c < 160; ++c)
        {
            // Coloring the point with the corrispondent point in the rectified image
            // Unsafe Zone
            if(cloud_frame.at<cv::Vec3b>(r,c)[0] == 255)
            {

              pcl::PointXYZRGB p;
              // The coordinate of the point is taken from the depth map
              // Y and Z  taken negative to immediately visualize the cloud in the right way
              p.x = r;
              p.y = c;
              p.z = 0;

              // Color point for unsafe zones
              p.r = 255;
              p.g = 0;
              p.b = 0;
              cloud->points.push_back(p);
            } 
        }
    }
    cloud->width = (int) cloud->points.size();
    cloud->height = 1;

    pub.publish(cloud);
    
    cv::imshow("Perspective Mapped Frame", output_frame);
    cv::waitKey(1);
  }
}