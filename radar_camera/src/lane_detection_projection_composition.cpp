


#include "../include/radar_camera/lane_detection_projection.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);

	rclcpp::executors::SingleThreadedExecutor exec;
	rclcpp::NodeOptions options;

	auto lane_detection_projection_node = std::make_shared<CAMERA::lane_detection_projection>(options);
	exec.add_node(lane_detection_projection_node->get_node_base_interface());

	RCLCPP_INFO(lane_detection_projection_node->get_logger(), "This is my lane_detection_projection");

	exec.spin();

	rclcpp::shutdown();

	return 0;
}