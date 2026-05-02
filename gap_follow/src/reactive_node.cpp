#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
using std::placeholders::_1;

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // Create ROS publishers and subscribers
        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 2);
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 2, std::bind(&ReactiveFollowGap::lidar_callback, this, _1)
        );
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

    float deg_to_rad(float degree) {
        return degree * M_PI / 180.0;
    }

    float rad_to_deg(float rad) {
        return rad * 180.0 / M_PI;
    }


    // constants to modifity for optimization
    float degree_threshold = deg_to_rad(25.0); // constant

    float base_speed = 2.4;  // base speed when going straight
    float min_speed = 1.2;   // minimum speed during a sharp turn
    float max_steering_angle = deg_to_rad(20.0); // max steering angle allowed
    // Potential field parameters
    float attraction_gain = 1.0;    //weight attractive force to the goal
    float repulsion_gain = 0.5;     // weight repulsive force from obstacles
    float repulsion_distance = 1.0; // Distance threshold for repulsive force method

void preprocess_lidar(std::vector<float>& ranges, int count_to_check)
{   
    int threshold_high = 10; // Threshold to filter out high range values

    // Loop through all elements in the ranges vector
    for (int i = 0; i < ranges.size(); i++) {
        // If the range value is greater than the threshold, set it to INFINITY
        if (ranges[i] > threshold_high) {
            ranges[i] = INFINITY;
        }

        // If the index is outside the central region defined by count_to_check,
        // set the value to INFINITY
        if (i < ranges.size() / 2 - count_to_check || i > ranges.size() / 2 + count_to_check) {
            ranges[i] = INFINITY;
        }

    }
}

    void find_max_gap(std::vector<float>& ranges, int* indice, int* return_size)
    {    
        int max_size = 0;
        int size = 1;
        int i = 0;
        int max_index = 0;
        int new_index = 0;
        bool new_gap = true;

        while (i < ranges.size()) {
            if (ranges[i] > 0.1 && ranges[i] != INFINITY) {
                if (new_gap) {
                    new_index = i;
                    new_gap = false;
                }
                // still in gap
                i++;
                size++;
            } else {
                // gap is over
                if (size > max_size) {
                    max_size = size;
                    max_index = new_index;
                }
                size = 1;
                i++;// keep increasing to find new index
                new_gap = true;
            }
        }

        *indice = max_index;
        *return_size = max_size;
        // RCLCPP_INFO(this->get_logger(), "Max Index: %d, size: %d", max_index, max_size);
    }

    void find_best_point(std::vector<float>& ranges, int& indice, int size)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        float furthest = 0.0;
        int index = indice;
        for (int i = index; i < index + size; i++) {
            if (ranges[i] > furthest) {
                furthest = ranges[i];
                indice = i;
            }
        }
        return;
    }
    bool once = true;
   //

///
// The potential_field_navigation function 
// Features:
// Add weights to obsticals on path to avoid them, there is no weights added for clear path
// the goal is set to the angle of stright ahead or 0 
//
//Arguments: 1 argument - constant laser scan message
//
// const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg
///
    void potential_field_navigation(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        float goal_angle = 0.0; // Goal is straight ahead
        float steering_angle = 0.0; //the var used for the angle to steer
        float total_repulsive_f = 0.0; // our var for total repulse force
        float total_repulsive_a = 0.0;// our var for total repulse angle 

        // Calculate repulsive forces from obstacles
    
        for (int i = 0; i < scan_msg->ranges.size(); i++) {
            // scans every message recived
            float range = scan_msg->ranges[i];
            // obstacles that are within the repulsion distance and have valid ranges
            bool is_obstacle_nearby = false; // var for if there is an obsticle nearby
            bool is_range_valid = false; // var to check for valid or invalid data
            // The range is valid (> 0)
            if (range > 0.0){
                is_range_valid = true;
            }else{
                is_range_valid = false;
            }
            if (range < repulsion_distance) {
                is_obstacle_nearby = true;
            }else{
                is_obstacle_nearby = false;
            }

            //lets varifiy both conditions so we can do the calculations
            if (is_obstacle_nearby && is_range_valid) {
                // Calculate the angle of the current point in the LiDAR scan
                float angle = scan_msg->angle_min + i * scan_msg->angle_increment;

                
                //Repulsive force = repulsion_gain * ( (1 / range) - ( 1 / repulsion_distance) )
                // Calculate the repulsive force based on the distance to the obstacle
                float repulsive_force = repulsion_gain * (1.0 / range - 1.0 / repulsion_distance);

                //if we have a repulsive force then we can add it to the totals same with angle 
                if (repulsive_force > 0) {
                    // Accumulate the total repulsive force magnitude
                    total_repulsive_f = total_repulsive_f + repulsive_force;

                    // Accumulate the weighted angle of the repulsive forces
                    total_repulsive_a = total_repulsive_a + (repulsive_force * angle);
                }
            }
        }

        // Calculate final steering angle using both attractive and repulsive forces
        // if there is an obstical then we need to steer away from it
        if (total_repulsive_f > 0) {
            steering_angle = (attraction_gain * goal_angle - total_repulsive_a / total_repulsive_f);
        } else {
            //otherwise we can keep steering stright
            steering_angle = goal_angle;
        }

        // Limit the steering angle
        if (steering_angle > max_steering_angle) {
            steering_angle = max_steering_angle;
        } else if (steering_angle < -max_steering_angle) {
            steering_angle = -max_steering_angle;
        }

        // Adjust speed based on steering angle
        float speed = base_speed - (std::abs(steering_angle) / max_steering_angle) * (base_speed - min_speed);
        speed = std::max(min_speed, std::min(base_speed, speed));  // Limit speed between min_speed and base_speed

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;

        // Debug information
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
                             "Steering Angle (Potential Field): %f, Speed: %f", rad_to_deg(steering_angle), speed);

        publisher_->publish(drive_msg);
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // Uncomment the method you want to use:

        // Method 1: Reactive Follow Gap
        follow_gap_navigation(scan_msg);

        // Method 2: Potential Field Navigation
        //potential_field_navigation(scan_msg);
    }

    //Follow Gap method
    void follow_gap_navigation(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        std::vector<float> ranges = scan_msg->ranges;
        int count_to_check = (degree_threshold / scan_msg->angle_increment); 
        int startingIndex = ranges.size() / 2 - count_to_check;

        preprocess_lidar(ranges, count_to_check);
        
        // Find closest point to LiDAR (within 20 degrees)
        float closest_point = INFINITY;
        int closest_point_index = 0;

        for (int i = startingIndex; i < startingIndex + (2 * count_to_check); i++) {
            if (ranges[i] < closest_point) {
                closest_point = ranges[i];
                closest_point_index = i;
            }
        }

        // Eliminate all points inside 'bubble' (set them to zero) 
        float rb = 0.3; // constant for now
        float angle_bubble = atan(rb / closest_point);
        int indices_to_adjust = (int)std::ceil(angle_bubble / scan_msg->angle_increment);
        for (int i = closest_point_index - indices_to_adjust; i < closest_point_index + indices_to_adjust; i++) {
            ranges[i] = 0.0;
        }
        
        // Find max length gap 
        int max_gap_index = 0;
        int max_gap_size = 0;
        find_max_gap(ranges, &max_gap_index, &max_gap_size);

        // Find the best point in the gap 
        int best_point_index = max_gap_index;
        find_best_point(ranges, best_point_index, max_gap_size);

        // Publish Drive message
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        float steering_angle = scan_msg->angle_min + (float)best_point_index * scan_msg->angle_increment;
        
        // Adjust speed based on steering angle
        float speed = base_speed - (std::abs(steering_angle) / max_steering_angle) * (base_speed - min_speed);
        speed = std::max(min_speed, std::min(base_speed, speed));  // Limit speed between min_speed and base_speed
        
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = steering_angle;

        if (steering_angle > max_steering_angle) {
            drive_msg.drive.steering_angle = max_steering_angle;
        } else if (steering_angle < -max_steering_angle) {
            drive_msg.drive.steering_angle = -max_steering_angle;
        }

        // Debug information
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 250,
                             "Best Point Index: %d, Steering Angle: %f, Speed: %f , Max Gap Index: %d", best_point_index,
                             rad_to_deg(drive_msg.drive.steering_angle), drive_msg.drive.speed, max_gap_index);

        publisher_->publish(drive_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}