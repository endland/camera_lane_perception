#ifndef COMPOSITION__LANE_DETECTION_PROJECTION_COMPONENT_HPP_
#define COMPOSITION__LANE_DETECTION_PROJECTION_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <map>
#include <math.h>

#define _USE_MATH_DEFINES

#include <opencv2/opencv.hpp>

#include "../include/radar_camera/matrixDefines.h"


#include "lane_msgs/msg/lane_marking.hpp"
#include "lane_msgs/msg/lane_marking_array_both.hpp"
#include "lane_msgs/msg/lane_marking_projected.hpp"
#include "lane_msgs/msg/lane_marking_projected_array_both.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_factory.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/timer.hpp>
#include <rmw/qos_profiles.h>

#include <rclcpp/strategies/message_pool_memory_strategy.hpp>
#include <rclcpp/strategies/allocator_memory_strategy.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <std_msgs/msg/string.hpp>



typedef unsigned char ubyte;
typedef unsigned short int uword;

#define BOOST_BIND_NO_PLACEHOLDERS

namespace CAMERA
{
	class lane_detection_projection : public rclcpp_lifecycle::LifecycleNode
	{
	public:
		lane_detection_projection(const rclcpp::NodeOptions& options);

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
			const rclcpp_lifecycle::State& );

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
			const rclcpp_lifecycle::State& );

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
			const rclcpp_lifecycle::State& );

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
			const rclcpp_lifecycle::State& );

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
			const rclcpp_lifecycle::State&);

		rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
			const rclcpp_lifecycle::State& );

		

		void laneMarkingCallback(lane_msgs::msg::LaneMarkingArrayBoth::SharedPtr msg);



	private:
		rclcpp::QoS qos{10};

		std::string pub_projected_marking_array_topic_name = "/lane_markings_left_right_projected";
		std::string sub_marking_topic_name = "/lane_markings_left_right";
		
		rclcpp_lifecycle::LifecyclePublisher<lane_msgs::msg::LaneMarkingProjectedArrayBoth>::SharedPtr projected_markings_publisher_;
		rclcpp::Subscription<lane_msgs::msg::LaneMarkingArrayBoth>::SharedPtr marking_subscriber_;

		//std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<lane_msgs::msg::LaneMarkingProjectedArrayBoth>> projected_markings_publisher_;
		//std::shared_ptr<rclcpp::Subscription<lane_msgs::msg::LaneMarkingArrayBoth>> marking_subscriber_;

		//std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

	};
}

#endif