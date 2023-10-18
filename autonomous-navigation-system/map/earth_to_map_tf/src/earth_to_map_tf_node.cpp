#include "earth_to_map_tf/earth_to_map_tf.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EarthToMapTfNode>("earth_to_map_tf_publisher");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}