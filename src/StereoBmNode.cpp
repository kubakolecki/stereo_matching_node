#include "stereo_bm/StereoBmNode.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <opencv2/opencv.hpp>

#include <limits>
#include <chrono>
#include <iostream>
#include <fstream>
#include <iomanip>


StereoBmNode::StereoBmNode(): rclcpp::Node("stereo_bm_node")
{
    auto paramImageInputTopicNameDescription = rcl_interfaces::msg::ParameterDescriptor{};
    
    auto paramDoSaveDisparityMapsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramDoSaveDepthMapsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramDoSaveLocalPointCloudsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramDoSaveGlobalPointCloudsDescription = rcl_interfaces::msg::ParameterDescriptor{};

    auto paramPathToSaveDisparityMapDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramPathToSaveDepthMapsDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramPathToSaveLocalPointCloudDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramPathToSaveGlobalPointCloudDescription = rcl_interfaces::msg::ParameterDescriptor{};

    auto paramRoiRowMinDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramRoiRowMaxDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramRoiColMinDescription = rcl_interfaces::msg::ParameterDescriptor{};
    auto paramRoiColMaxDescription = rcl_interfaces::msg::ParameterDescriptor{};

    paramImageInputTopicNameDescription.description = "Input topic name for stereo image";
    
    paramDoSaveDisparityMapsDescription.description = "Whether to save disparity maps to disk";
    paramDoSaveDepthMapsDescription.description = "Whether to save depth maps to disk";
    paramDoSaveLocalPointCloudsDescription.description = "Whether to save local point cloud";
    paramDoSaveGlobalPointCloudsDescription.description = "Whether to save global point cloud";

    paramPathToSaveDisparityMapDescription.description = "Path to save disparity maps"; 
    paramPathToSaveDepthMapsDescription.description = "Path to save depth maps";
    paramPathToSaveLocalPointCloudDescription.description = "Path to save local point cloud";
    paramPathToSaveGlobalPointCloudDescription.description = "Path to save point cloud transformed to wordl coordinate system";

    paramRoiRowMinDescription.description = "Minimum row index for region of interest (ROI)";
    paramRoiRowMaxDescription.description = "Maximum row index for region of interest (ROI)";
    paramRoiColMinDescription.description = "Minimum column index for region of interest (ROI)";
    paramRoiColMaxDescription.description = "Maximum column index for region of interest (ROI)";

    this->declare_parameter("input_topic_name", "/orbslam3/georeferenced_stereo_image", paramImageInputTopicNameDescription);

    this->declare_parameter("do_save_parallax_maps", true, paramDoSaveDisparityMapsDescription);
    this->declare_parameter("do_save_depth_maps", true, paramDoSaveDepthMapsDescription); 
    this->declare_parameter("do_save_local_point_clouds", true, paramDoSaveLocalPointCloudsDescription); 
    this->declare_parameter("do_save_global_point_clouds", true, paramDoSaveGlobalPointCloudsDescription);

    this->declare_parameter("path_to_save_parallax_map", std::string("/path/to/save/prallax/maps"), paramPathToSaveDisparityMapDescription);
    this->declare_parameter("path_to_save_depth_maps", std::string("/path/to/save/depth/maps"), paramPathToSaveDepthMapsDescription);
    this->declare_parameter("path_to_save_local_point_clouds", std::string("/path/to/save/local/point/clouds"), paramPathToSaveLocalPointCloudDescription);
    this->declare_parameter("path_to_save_global_point_clouds", std::string("/path/to/save/local/point/clouds"), paramPathToSaveGlobalPointCloudDescription);

    this->declare_parameter("roi_row_min", 0, paramRoiRowMinDescription);
    this->declare_parameter("roi_row_max", 1000, paramRoiRowMaxDescription);
    this->declare_parameter("roi_col_min", 0, paramRoiColMinDescription);
    this->declare_parameter("roi_col_max", 1000, paramRoiColMaxDescription);

    globalPointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("stereo_bm/global_point_cloud", rclcpp::QoS(20));
    localPointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("stereo_bm/local_point_cloud", rclcpp::QoS(20));

    m_roi = cv::Rect(
        this->get_parameter("roi_col_min").as_int(),
        this->get_parameter("roi_row_min").as_int(),
        this->get_parameter("roi_col_max").as_int() - this->get_parameter("roi_col_min").as_int(),
        this->get_parameter("roi_row_max").as_int() - this->get_parameter("roi_row_min").as_int());



    m_p1 = 8 * 1 * m_blockSize * m_blockSize;
    m_p2 = 2*32 * 1 * m_blockSize * m_blockSize;

    //m_p1 = 1 * 1 * m_blockSize * m_blockSize;
    //m_p2 = 4 * 1 * m_blockSize * m_blockSize;

    //m_p1 = 0;
   // m_p2 = 0;

    stereoSgBm = cv::StereoSGBM::create(
        m_minDisparity,
        m_numDisparities,
        m_blockSize,
        m_p1,
        m_p2,
        m_disp12MaxDiff,
        m_preFilterCap,
        m_uniquenessRatio,
        m_speckleWindowSize,
        m_speckleRange,
        cv::StereoSGBM::MODE_HH);

    m_pathToSaveParallaxMaps = std::filesystem::path{this->get_parameter("path_to_save_parallax_map").as_string()};
    m_pathToSaveDepthMaps = std::filesystem::path{this->get_parameter("path_to_save_depth_maps").as_string()};
    m_pathToSaveLocalPoitCloud = std::filesystem::path{this->get_parameter("path_to_save_local_point_clouds").as_string()};
    m_pathToSaveGlobalPoitCloud = std::filesystem::path{this->get_parameter("path_to_save_global_point_clouds").as_string()};

    if (this->get_parameter("do_save_parallax_maps").as_bool() && !std::filesystem::exists(m_pathToSaveParallaxMaps))
    {
        std::filesystem::create_directories(m_pathToSaveParallaxMaps);
    }

    if (this->get_parameter("do_save_depth_maps").as_bool() && !std::filesystem::exists(m_pathToSaveDepthMaps))
    {
        std::filesystem::create_directories(m_pathToSaveDepthMaps);       
    }
    
    if (this->get_parameter("do_save_local_point_clouds").as_bool() && !std::filesystem::exists(m_pathToSaveLocalPoitCloud))
    {
        std::filesystem::create_directories(m_pathToSaveLocalPoitCloud);       
    }

    if (this->get_parameter("do_save_global_point_clouds").as_bool() && !std::filesystem::exists(m_pathToSaveGlobalPoitCloud))
    {
        std::filesystem::create_directories(m_pathToSaveGlobalPoitCloud);       
    }

    auto topicCallback = [this](const ros_common_messages::msg::GeoreferencedStereoImage::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Received stereo image");
        auto startParallaxComputation {std::chrono::steady_clock::now()};
        const auto imageLeft {cv_bridge::toCvCopy(msg->image_left, "mono8")};
        const auto imageRight {cv_bridge::toCvCopy(msg->image_right, "mono8")};
        const auto imageLeftRoi {imageLeft->image(m_roi) };
        const auto imageRightRoi {imageRight->image(m_roi) };
        cv::Mat parallaxMap;
        cv::Mat floatParallaxMap;
        //stereoSgBm->compute(imageLeft->image, imageRight->image, parallaxMap);
        stereoSgBm->compute(imageLeftRoi, imageRightRoi, parallaxMap);
        parallaxMap.convertTo(floatParallaxMap, CV_32F, 1.0f / 16.0f);

        auto endParallaxComputation {std::chrono::steady_clock::now()};
        const auto durationParallaxComputation {std::chrono::duration_cast<std::chrono::milliseconds>(endParallaxComputation - startParallaxComputation)};
        RCLCPP_INFO(this->get_logger(), "Computed parallax map in %ld ms", durationParallaxComputation.count());

        if (this->get_parameter("do_save_parallax_maps").as_bool())
        {
            const auto pathFileParallaxMap{m_pathToSaveParallaxMaps / (stampToString(msg->pose.header.stamp) + ".tif")};
            cv::imwrite(pathFileParallaxMap.string(), floatParallaxMap);
            RCLCPP_INFO(this->get_logger(), "Saved parallax map to: %s", pathFileParallaxMap.string().c_str());
        }



        auto startProjectingTo3D {std::chrono::steady_clock::now()};

        cv::Matx44d Q = cv::Matx44d::eye();
        Q(2, 2) = 0.0;
        Q(0, 3) = -(msg->camera_matrix_left.data[2] - m_roi.x); // cx
        Q(1, 3) = -(msg->camera_matrix_left.data[5] - m_roi.y); // cy
        Q(2, 3) = msg->camera_matrix_left.data[0]; // fx
        Q(3, 2) = 1.0 / msg->right_to_left_transformation_matrix.data[3]; // 1/baseline
        Q(3, 3) = (msg->camera_matrix_left.data[2] - msg->camera_matrix_right.data[2])/msg->right_to_left_transformation_matrix.data[3]; // cx_left - cx_right

        std::cout << "Q matrix: " << std::endl << Q << std::endl;

  
        //cv::Mat parallaxMapRoi {floatParallaxMap(m_roi)};
        cv::Mat pointCloudInCameraFrame;
        cv::reprojectImageTo3D(floatParallaxMap, pointCloudInCameraFrame, Q, true);
        setInfValues(pointCloudInCameraFrame, m_depthLimit);

        std::cout<<"rows, cols, channels: " << pointCloudInCameraFrame.rows << " " << pointCloudInCameraFrame.cols <<" " << pointCloudInCameraFrame.channels() << std::endl;

        auto endProjectingTo3D {std::chrono::steady_clock::now()};
        const auto durationProjectionTo3D {std::chrono::duration_cast<std::chrono::milliseconds>(endProjectingTo3D - startProjectingTo3D)};
        RCLCPP_INFO(this->get_logger(), "Computed projection to 3D in %ld ms", durationProjectionTo3D.count());

        
        const auto startSavingToFileLocal {std::chrono::steady_clock::now()};
        if (this->get_parameter("do_save_local_point_clouds").as_bool())
        {
            const auto pathFileLocalPointCloud{m_pathToSaveLocalPoitCloud / (stampToString(msg->pose.header.stamp) + "_camera_cs.txt")};     
            //cv::Mat imageLeftRoi {imageLeft->image(m_roi) };
            savePointCloudToFile(pointCloudInCameraFrame, imageLeftRoi, pathFileLocalPointCloud);

        }
        const auto endSavingToFileLocal {std::chrono::steady_clock::now()};
        const auto durationSavingToFileLocal {std::chrono::duration_cast<std::chrono::milliseconds>(endSavingToFileLocal - startSavingToFileLocal)};
        RCLCPP_INFO(this->get_logger(), "Point cloud saved to text file in %ld ms", durationSavingToFileLocal.count());

        const auto startTransformingToWorld {std::chrono::steady_clock::now()};
        const auto se3 {poseStampedToSE3(msg->pose)};
        cv::Mat pointCloudInWorldFrame;
        cv::perspectiveTransform(pointCloudInCameraFrame, pointCloudInWorldFrame, se3);
        const auto endTransformingToWorld {std::chrono::steady_clock::now()};
        const auto durationTransformingToWorlds {std::chrono::duration_cast<std::chrono::milliseconds>(endTransformingToWorld - startTransformingToWorld)};
        RCLCPP_INFO(this->get_logger(), "Point cloud transformed to wrold coordinate system in %ld ms", durationTransformingToWorlds.count());

        const auto startSavingToFileGlobal {std::chrono::steady_clock::now()};
        if (this->get_parameter("do_save_global_point_clouds").as_bool())
        {
            const auto pathFileGlobalPointCloud{m_pathToSaveLocalPoitCloud / (stampToString(msg->pose.header.stamp) + "_world_cs.txt")};     
            //cv::Mat imageLeftRoi {imageLeft->image(m_roi) };
            savePointCloudToFile(pointCloudInWorldFrame, imageLeftRoi, pathFileGlobalPointCloud);

        }
        const auto endSavingToFileGlobal {std::chrono::steady_clock::now()};
        const auto durationSavingToFileGlobal {std::chrono::duration_cast<std::chrono::milliseconds>(endSavingToFileGlobal - startSavingToFileGlobal)};
        RCLCPP_INFO(this->get_logger(), "Point cloud saved to text file in %ld ms", durationSavingToFileGlobal.count());

        const auto globalPointCloudMessage {createPointCloud2(pointCloudInWorldFrame, imageLeftRoi, msg->pose.header)};
        globalPointCloudPublisher->publish(globalPointCloudMessage);

       

        //std::cout << "se3: \n";
        //std::cout << se3 << std::endl;

    };
    
    stereoImageSubscription = this->create_subscription<ros_common_messages::msg::GeoreferencedStereoImage>(this->get_parameter("input_topic_name").as_string(), rclcpp::QoS(10), topicCallback);



}

std::string StereoBmNode::stampToString(builtin_interfaces::msg::Time stamp) const
{
    std::ostringstream oss;
    oss << std::setw(9) << std::setfill('0') << stamp.nanosec;
    std::string timeInformationStr {std::to_string(stamp.sec) + "_" +  oss.str()};
    return timeInformationStr;
}

cv::Mat StereoBmNode::poseStampedToSE3(const geometry_msgs::msg::PoseStamped& pose) const
{
    tf2::Quaternion q{
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w};

    tf2::Matrix3x3 R{q};

    cv::Mat T = cv::Mat::eye(4, 4, CV_64F);

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            T.at<double>(i,j) = R[i][j];
        }
    }

    T.at<double>(0,3) = pose.pose.position.x;
    T.at<double>(1,3) = pose.pose.position.y;
    T.at<double>(2,3) = pose.pose.position.z;

    return T;
}

void StereoBmNode::setInfValues(cv::Mat& coordinates, float depthLimit) const
{
    const auto inf {std::numeric_limits<float>::infinity()};

    for (int row = 0; row < coordinates.rows; ++row)
    {
        cv::Vec3f* rowPtr {coordinates.ptr<cv::Vec3f>(row)};

        for (int col = 0; col < coordinates.cols; ++col)
        {
            cv::Vec3f& point {rowPtr[col]};

            if (point[2] >= depthLimit)
            {
                point[0] = inf;
                point[1] = inf;
                point[2] = inf;
            }


        }
    }

}


void StereoBmNode::savePointCloudToFile(const cv::Mat& coordinates, const cv::Mat& image,  const std::filesystem::path& pathToTextFile ) const
{
    if ((coordinates.rows != image.rows) || (coordinates.cols != image.cols))
    {
        RCLCPP_FATAL(this->get_logger(), "Inconsistent dimensions of coordinates array and the image while trying to save point cloud to file");
    }
    
    std::ofstream outputFile{pathToTextFile};

    outputFile << std::fixed << std::setprecision(5);

    for (int row = 0; row < coordinates.rows; ++row)
    {
        const cv::Vec3f* rowPtr {coordinates.ptr<cv::Vec3f>(row)};
        const uchar* rowPtrImage {image.ptr<uint8_t>(row)};

        for (int col = 0; col < coordinates.cols; ++col)
        {
            const cv::Vec3f& point {rowPtr[col]};
                 
            if (std::isnan(point[0]) || std::isnan(point[1]) ||  std::isnan(point[2]))
            {
                continue;
            }

            if (std::isinf(point[0]) || std::isinf(point[1]) ||  std::isinf(point[2]))
            {
                continue;
            }

            if (point[0] == 0.0f && point[1] == 0.0f && point[2] == 0.0f )
            {
                continue;
            }

            const auto intensity {static_cast<int>(rowPtrImage[col])};
            outputFile << point[0] <<","<<point[1]<<"," << point[2] <<",";
            outputFile << intensity<<"," << intensity <<","<<intensity << "\n";
            
            // p[0] = x
            // p[1] = y
            // p[2] = z
        }
    }
}


sensor_msgs::msg::PointCloud2 StereoBmNode::createPointCloud2(const cv::Mat& coordinates, const cv::Mat& intensity, const std_msgs::msg::Header& header) const
{
    // Sanity checks
    CV_Assert(coordinates.type() == CV_32FC3);
    CV_Assert(intensity.type() == CV_8UC1);
    CV_Assert(coordinates.size() == intensity.size());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;

    cloud.height = coordinates.rows;
    cloud.width = coordinates.cols;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::UINT8);

    modifier.resize(cloud.width * cloud.height);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t> iter_i(cloud, "intensity");

    for (int r = 0; r < coordinates.rows; ++r)
    {
        const cv::Vec3f* coord_row = coordinates.ptr<cv::Vec3f>(r);
        const uint8_t* intensity_row = intensity.ptr<uint8_t>(r);

        for (int c = 0; c < coordinates.cols;
             ++c, ++iter_x, ++iter_y, ++iter_z, ++iter_i)
        {
            const cv::Vec3f& point = coord_row[c];

            if (std::isnan(point[0]) || std::isnan(point[1]) ||  std::isnan(point[2]))
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                *iter_i = 0;
                continue;
            }

            if (std::isinf(point[0]) || std::isinf(point[1]) ||  std::isinf(point[2]))
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                *iter_i = 0;
                continue;
            }

            if (point[0] == 0.0f && point[1] == 0.0f && point[2] == 0.0f )
            {
                *iter_x = std::numeric_limits<float>::quiet_NaN();
                *iter_y = std::numeric_limits<float>::quiet_NaN();
                *iter_z = std::numeric_limits<float>::quiet_NaN();
                *iter_i = 0;
                continue;
            }

            *iter_x = point[0];
            *iter_y = point[1];
            *iter_z = point[2];
            *iter_i = intensity_row[c];
        }
    }

    cloud.is_dense = false; // set true if no invalid points
    cloud.is_bigendian = false;

    return cloud;
}