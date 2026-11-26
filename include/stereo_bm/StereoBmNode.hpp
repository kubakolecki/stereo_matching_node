#pragma once

#include "ros_common_messages/msg/georeferenced_stereo_image.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <opencv2/calib3d.hpp>

#include <string>
#include <filesystem>

class StereoBmNode : public rclcpp::Node
{
public:
  StereoBmNode();

private:
    rclcpp::Subscription<ros_common_messages::msg::GeoreferencedStereoImage>::SharedPtr stereoImageSubscription;

    int m_minDisparity{0};
    int m_numDisparities{128};
    int m_blockSize{3};
    int m_p1{0};
    int m_p2{0};
    int m_disp12MaxDiff{0};
    int m_preFilterCap{31};
    int m_uniquenessRatio{30};
    int m_speckleWindowSize{150};
    int m_speckleRange{1};
    cv::Rect m_roi;

    float m_depthLimit{6.0f};



    cv::Ptr<cv::StereoSGBM> stereoSgBm;

    std::filesystem::path m_pathToSaveParallaxMaps;
    std::filesystem::path m_pathToSaveDepthMaps;
    std::filesystem::path m_pathToSaveLocalPoitCloud;
    std::filesystem::path m_pathToSaveGlobalPoitCloud;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalPointCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr localPointCloudPublisher;

    std::string stampToString(builtin_interfaces::msg::Time stamp) const;

    void savePointCloudToFile(const cv::Mat& coordinates, const cv::Mat& image, const std::filesystem::path& pathToTextFile ) const;
    
    void setInfValues(cv::Mat& coordinates, float depthLimit) const;
    cv::Mat poseStampedToSE3(const geometry_msgs::msg::PoseStamped& pose) const;
    sensor_msgs::msg::PointCloud2 createPointCloud2(const cv::Mat& coordinates, const cv::Mat& intensity, const std_msgs::msg::Header& header) const;

};