#include "stereo_bm/StereoBmNode.hpp"

int main(int argc, char * argv[] )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoBmNode>());
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}