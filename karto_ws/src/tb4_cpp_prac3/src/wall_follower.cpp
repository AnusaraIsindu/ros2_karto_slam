#include <chrono>  
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;


class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        /*
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */

        auto wall_side_desc = rcl_interfaces::msg::ParameterDescriptor{};
        wall_side_desc.description = "A positive value indicates that the wall will be on the left side of the robot, otherwise on the right";
        auto buffer_zone_desc = rcl_interfaces::msg::ParameterDescriptor{};
        buffer_zone_desc.description = "A positive value used to determine whether the tracking control is on or off";

        this->declare_parameter<double>("following_distance", 0.7);
        this->declare_parameter<int8_t>("wall_side", 1, wall_side_desc);
        this->declare_parameter<double>("buffer_zone", 0.4, buffer_zone_desc);
        this->declare_parameter<double>("forward_velocity", 0.5);
        this->declare_parameter<double>("angle_control_gain_1", 1.0);
        this->declare_parameter<double>("angle_control_gain_2", 1.0);
        this->declare_parameter<double>("distance_control_gain", 0.5);

        this->get_parameter("following_distance", following_distance_);
        this->get_parameter("wall_side", wall_side_);
        this->get_parameter("buffer_zone", buffer_zone_);
        this->get_parameter("forward_velocity", forward_velocity_);
        this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
        this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
        this->get_parameter("distance_control_gain", distance_control_gain_);
        
        RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
        RCLCPP_INFO(this->get_logger(), "wall_side: %ld", wall_side_);
        RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
        RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
        RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
        RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);
        
        if(wall_side_>0)
            following_angle_ = PI/2;
        else
            following_angle_ = -PI/2;

        dyn_params_handler_ = this->add_on_set_parameters_callback(
            std::bind(
            &WallFollower::dynamicParametersCallback,
            this, std::placeholders::_1));

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
             "/cmd_vel",
             rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher. Same as person follower
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber. Same as person follower
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
  
    //this makes a shared pointer that will keep the handle for the param change callback
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    /** 
    * @brief Callback executed when a parameter change is detected
    * @param event ParameterEvent message
    */

    rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);
    double following_angle_;
    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    double distance_control_gain_;
};
void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    
    auto block = [] (float a, float b) {
        if (a < 0.2) return false;
        if (b < 0.2) return true;
        return a < b;
    };
    auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end(), block);
    float min_value = *min_distance;
    int min_index = std::distance(scan_msg->ranges.begin(), min_distance);

    float angle_L = (scan_msg->angle_min) + (min_index * scan_msg->angle_increment); 
    float lidar_to_robot_offset = PI/2.0; // 90 degrees offset
    float angle_R = angle_L + lidar_to_robot_offset;
    angle_R = std::atan2(std::sin(angle_R), std::cos(angle_R)); // Normalize angle

    geometry_msgs::msg::Twist cmd_vel_msg;
    
    // Initialize to zero (prevents uninitialized values)
    cmd_vel_msg.linear.x = 0.0;
    cmd_vel_msg.linear.y = 0.0;
    cmd_vel_msg.linear.z = 0.0;
    cmd_vel_msg.angular.x = 0.0;
    cmd_vel_msg.angular.y = 0.0;
    cmd_vel_msg.angular.z = 0.0;

    if (min_value < 12.0) {
        // Phase 1: Approaching the wall
        if (min_value > (following_distance_ + buffer_zone_)) {
            if (abs(angle_R) > PI/4.0) {
                // Turn towards wall
                cmd_vel_msg.linear.x = 0.0;  // Stop moving forward while turning
                if (angle_R > PI/4.0)
                    cmd_vel_msg.angular.z = 0.8; 
                else
                    cmd_vel_msg.angular.z = -0.8;
            }
            else {
                // Move straight towards wall
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_msg.linear.x = forward_velocity_;
            }
        }
        // Phase 2: Wall following mode
        else {
            // Calculate control signals
            double angular_velocity;
            if (wall_side_ > 0) {
                angular_velocity = angle_control_gain_1_ * (angle_R - following_angle_) 
                                 + angle_control_gain_2_ * (min_value - following_distance_);
            }
            else {
                angular_velocity = angle_control_gain_1_ * (angle_R - following_angle_) 
                                 - angle_control_gain_2_ * (min_value - following_distance_);
            }
            
            double linear_velocity = forward_velocity_ 
                                   + distance_control_gain_ * (min_value - following_distance_);
            
            // Clamp angular velocity to safe range [-1.0, 1.0] rad/s
            const double MAX_ANGULAR_VEL = 1.0;
            const double MIN_ANGULAR_VEL = -1.0;
            cmd_vel_msg.angular.z = std::max(MIN_ANGULAR_VEL, 
                                             std::min(angular_velocity, MAX_ANGULAR_VEL));
            
            // Clamp linear velocity to safe range [0.0, 0.22] m/s
            const double MAX_LINEAR_VEL = 0.22;  // TurtleBot4 safe max
            const double MIN_LINEAR_VEL = 0.0;
            cmd_vel_msg.linear.x = std::max(MIN_LINEAR_VEL, 
                                            std::min(linear_velocity, MAX_LINEAR_VEL));
            
            // ADDITIONAL SAFETY: Slow down during sharp turns
            if (std::abs(cmd_vel_msg.angular.z) > 0.6) {
                cmd_vel_msg.linear.x *= 0.5; // Reduce speed by 50% during sharp turns
            }
            
            // Emergency brake if too close to wall
            if (min_value < 0.15) {
                RCLCPP_WARN(this->get_logger(), "⚠️ TOO CLOSE TO WALL! Distance: %.2f m", min_value);
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.angular.z = (wall_side_ > 0) ? 0.5 : -0.5; // Turn away
            }
            
            // Log if limits were applied (for debugging)
            if (std::abs(angular_velocity) > MAX_ANGULAR_VEL || 
                linear_velocity > MAX_LINEAR_VEL || linear_velocity < MIN_LINEAR_VEL) {
                RCLCPP_DEBUG(this->get_logger(), 
                    "Velocity clamped: lin=(%.2f->%.2f) ang=(%.2f->%.2f)", 
                    linear_velocity, cmd_vel_msg.linear.x,
                    angular_velocity, cmd_vel_msg.angular.z);
            }
        }
    }
    else {
        // No object detected - move forward slowly
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "No Object is Detected");
        cmd_vel_msg.linear.x = 0.15;  // Reduced from 0.2 for safety
        cmd_vel_msg.angular.z = 0.0;
    }
    
    // Final safety check: Ensure no NaN or Inf values
    if (!std::isfinite(cmd_vel_msg.linear.x) || !std::isfinite(cmd_vel_msg.angular.z)) {
        RCLCPP_ERROR(this->get_logger(), "⚠️ INVALID VELOCITY DETECTED! Setting to zero.");
        cmd_vel_msg.linear.x = 0.0;
        cmd_vel_msg.angular.z = 0.0;
    }
    
    // Publish command
    cmd_vel_publisher_->publish(cmd_vel_msg);
}

rcl_interfaces::msg::SetParametersResult 
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = true;
    auto update_param = [this, &result](const rclcpp::Parameter &param) {
        const std::string &name = param.get_name();
		
		//parameter input control functions below
        if (name == "following_distance") { 
        	following_distance_ = param.as_double(); 
        }
        else if (name == "buffer_zone") { 
        	buffer_zone_ = param.as_double(); 
        }
        else if (name == "forward_velocity") { 
        	forward_velocity_ = param.as_double(); 
        }
        else if (name == "angle_control_gain_1") { 
        	angle_control_gain_1_ = param.as_double(); 
        }
        else if (name == "angle_control_gain_2") { 
        	angle_control_gain_2_ = param.as_double(); 
        }
        else if (name == "distance_control_gain") { 
        	distance_control_gain_ = param.as_double(); 
        }
        else if (name == "wall_side") { 
            wall_side_ = param.as_int();
            following_angle_ = (wall_side_ > 0) ? PI/2 : -PI/2; 
        }
        else {
            result.successful = false;
            return;
        }
    };

    for (auto parameter : parameters) {
        update_param(parameter);
    }

    return result;
}
    

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}
