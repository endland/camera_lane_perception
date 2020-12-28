#include "../include/radar_camera/lane_detection_projection.hpp"

//#include <ament_index_cpp/get_package_share_directory.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;


namespace CAMERA
{
	lane_detection_projection::lane_detection_projection(const rclcpp::NodeOptions& options)
	: rclcpp_lifecycle::LifecycleNode("lane_detection_projection", options)
	{

	}

	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_error(
        const rclcpp_lifecycle::State& )
        {
            RCUTILS_LOG_INFO_NAMED(get_name(), "on error is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_shutdown(
        const rclcpp_lifecycle::State& ) 
        {
            RCUTILS_LOG_INFO_NAMED(get_name(), "on_shutdown is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_configure(
        const rclcpp_lifecycle::State& )
        { 
            //###### Task3 ##############
            // Create Publisher
            RCUTILS_LOG_INFO_NAMED(get_name(), "on_configure is called.");

            //pub_ = this->create_publisher<std_msgs::msg::String>(pub_projected_marking_array_topic_name, qos);

            projected_markings_publisher_ = this->create_publisher<lane_msgs::msg::LaneMarkingProjectedArrayBoth>(pub_projected_marking_array_topic_name, qos);
            
            marking_subscriber_ = this->create_subscription<lane_msgs::msg::LaneMarkingArrayBoth>(sub_marking_topic_name, qos, std::bind(&lane_detection_projection::laneMarkingCallback, this, _1));
          

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;

        }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_activate(
        const rclcpp_lifecycle::State& )
        {
            //########### Task4 ###############
            // activate publisher
            projected_markings_publisher_->on_activate();
            
            RCUTILS_LOG_INFO_NAMED(get_name(), "on_activate() is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_deactivate(
        const rclcpp_lifecycle::State& )
        {
            RCUTILS_LOG_INFO_NAMED(get_name(), "on_deactivate() is called");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn lane_detection_projection::on_cleanup(
        const rclcpp_lifecycle::State& )
        {
            RCUTILS_LOG_INFO_NAMED(get_name(), "on cleanup is called.");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }


        void lane_detection_projection::laneMarkingCallback(lane_msgs::msg::LaneMarkingArrayBoth::SharedPtr msg)
        {
            lane_msgs::msg::LaneMarkingProjectedArrayBoth combined_lanes;

            if (msg->markings_left.size() > 0)
            {
                cv::Mat_<cv::Point2f> leftLanePointMatrix(1, msg->markings_left.size());
                cv::Mat undistortedLeftLanePointMatrixNormalised;

                for (int i=0; i < msg->markings_left.size(); i++)
                {
                    leftLanePointMatrix(i) = cv::Point2f(msg->markings_left[i].u, msg->markings_left[i].v);
                }
                undistortPoints(leftLanePointMatrix, undistortedLeftLanePointMatrixNormalised, intrinsic_calibration_matrix, cv::noArray());
                for (cv::MatIterator_<cv::Point2f> i = undistortedLeftLanePointMatrixNormalised.begin<cv::Point2f>(); i != undistortedLeftLanePointMatrixNormalised.end<cv::Point2f>(); ++i)
                {
                    float norm[] = {(*i).x, (*i).y, 1};
                    cv::Mat rotated_ray = cv::Mat_<float>(1, 3, norm) * rotation_matrix.inv();
                    cv::Mat rotated_normalised_ray = rotated_ray / rotated_ray.at<float>(2);
                    float xGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(0) + transform_matrix.at<float>(0);
                    float yGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(1) + transform_matrix.at<float>(1);
                    lane_msgs::msg::LaneMarkingProjected lmp;
                    lmp.x = xGround;
                    lmp.y = yGround;
                    lmp.z = 0;
                    combined_lanes.markings_left.push_back(lmp);
                }
            }

            if (msg->markings_right.size() > 0)
            {
                cv::Mat_<cv::Point2f> rightLanePointMatrix(1, msg->markings_right.size());
                cv::Mat undistortedRightLanePointMatrixNormalised;
                
                for (int i = 0; i < msg->markings_right.size(); i++)
                {
                    rightLanePointMatrix(i) = cv::Point2f(msg->markings_right[i].u, msg->markings_right[i].v);
                }

                undistortPoints(rightLanePointMatrix, undistortedRightLanePointMatrixNormalised, intrinsic_calibration_matrix, cv::noArray());

                for (cv::MatIterator_<cv::Point2f> i = undistortedRightLanePointMatrixNormalised.begin<cv::Point2f>(); i != undistortedRightLanePointMatrixNormalised.end<cv::Point2f>(); i++)
                {
                    float norm[] = {(*i).x, (*i).y, 1};
                    cv::Mat rotated_ray = cv::Mat_<float>(1, 3, norm) * rotation_matrix.inv();
                    cv::Mat rotated_normalised_ray = rotated_ray / rotated_ray.at<float>(2);

                    float xGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(0) + transform_matrix.at<float>(0);
                    float yGround = transform_matrix.at<float>(2) * rotated_normalised_ray.at<float>(1) + transform_matrix.at<float>(1);

                    lane_msgs::msg::LaneMarkingProjected lmp;
                    lmp.x = xGround;
                    lmp.y = yGround;
                    lmp.z = 0;
                    combined_lanes.markings_right.push_back(lmp);
                }
            }

        	projected_markings_publisher_->publish(combined_lanes);
        }


}