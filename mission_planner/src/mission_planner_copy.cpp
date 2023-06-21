#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <cmath>

using namespace ros;
using namespace std;

const int WAITING = 99;
const int TAKE_PHOTO = 1;
const int APPROACH = 2;
const int RETURN = 3;
const double ONE_METER_AWAY_BODY  = 0.6;
const double DISTANCE_OFFSET  = 3.0;
const double FOV_DEGREES = 69.4;
const double delta = 0.5;
bool first_time = true;
const double h = 1.0;
class MissionPlanner {
public:

    double current_attitude_mission[3];
    double angle = 0.0;
    double x_desired = 0.0;
    double y_desired = 0.0;
    double z_desired = h;

    int current_mode = WAITING;
    int current_target_index = 0;
    double target_person_distance = 0.0;
    bool save_image = false;
    double q[4];
    bool home_pos = true;
    Eigen::Vector2d offset;

    geometry_msgs::PoseStamped initial_pose;

    // Constructor definition
    MissionPlanner() {
        drone_pose_sub = nh.subscribe("mavros/vision_pose/pose", 10, &MissionPlanner::dronePoseCallback, this);
        image_sub = nh.subscribe("/camera/color/image_raw", 0.1, &MissionPlanner::imageCallback, this);
        body_size_sub = nh.subscribe("/posenet/body_size_out", 10, &MissionPlanner::bodySizeCallback, this);
        gesture_sub = nh.subscribe("/posenet/gesture_out", 10, &MissionPlanner::gestureCallback, this);
        position_sub = nh.subscribe("/posenet/position_out", 10, &MissionPlanner::positionsCallback, this);

        target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/photo_zone_pose", 10);
        // image_pub = nh.advertise<sensor_msgs::Image>("image_saver_topic", 10);
    }

    void QuaternionToEuler(double& roll, double& pitch, double& yaw){	
        // roll (x-axis rotation)
        double t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
        double t1 = +1.0 - 2.0 * (q[0] * q[0] + q[1]*q[1]);
        roll = std::atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        pitch = -std::asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
        double t4 = +1.0 - 2.0 * (q[1]*q[1] + q[2] * q[2]);
        yaw = std::atan2(t3, t4);
    }

    Eigen::Matrix2d create2DRotationMatrix(double yaw) {
        Eigen::Matrix2d rotationMatrix;
        rotationMatrix << cos(yaw), -sin(yaw),
                        sin(yaw),  cos(yaw);
        return rotationMatrix;
    }

    Eigen::Vector2d createVector(double D, double angle) {
        Eigen::Vector2d vec;
        vec << D*cos(angle),
            D*sin(angle);
        return vec;
    }

    double distance(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2) + std::pow(z2 - z1, 2));
    }


    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (home_pos) {
            initial_pose = *msg; // store the initial pose when drone is in WAITING mode
            home_pos = false;
        }
        current_drone_pose = *msg;
        q[0] = msg->pose.orientation.x; 
        q[1] = msg->pose.orientation.y; 
        q[2] = msg->pose.orientation.z; 
        q[3] = msg->pose.orientation.w;


        QuaternionToEuler(current_attitude_mission[0],current_attitude_mission[1],current_attitude_mission[2]);
    }


    void imageCallback(const sensor_msgs::Image::ConstPtr& img) {
        if(save_image){
            try {
                cv::Mat cv_image = cv_bridge::toCvShare(img, "bgr8")->image;
                cv::imwrite("/home/usrg/Pictures/team1_test0.jpg", cv_image);
                ROS_INFO("Image saved.");
                save_image = false;
                current_mode=WAITING;
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
            }
        }
        else {
            current_drone_image = *img;
        }
    }

    void bodySizeCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        // cout<<"one bodysize callback"<<endl;
        body_size_data = *msg;
        if (current_target_index < body_size_data.data.size()) {
            target_person_distance = ONE_METER_AWAY_BODY / body_size_data.data[current_target_index];
        } else {
            // ROS_WARN("Current target index out of bounds in body size data.");
        }
    }

    void gestureCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
        gesture_data = *msg;
        for (int idx = 0; idx < gesture_data.data.size(); idx++) {
            int gesture = gesture_data.data[idx];

            if (current_mode == WAITING) {
                if (gesture == TAKE_PHOTO) {
                    current_mode = TAKE_PHOTO;
                    ROS_INFO_STREAM("Mode: Took photo");
                } else if (gesture == APPROACH) {
                    current_mode = APPROACH;
                    ROS_INFO_STREAM("Mode: Follow");
                } else if (gesture == RETURN) {
                    current_mode = RETURN;
                    ROS_INFO_STREAM("Mode: Return home");
                } else {
                    if (first_time) {
                        x_desired = 0;
                        y_desired = 0;
                        z_desired = h;
                        first_time = false;
                    } else {
                        ROS_INFO_STREAM("Mode: Hovering in place");
                        x_desired = current_drone_pose.pose.position.x;
                        y_desired = current_drone_pose.pose.position.y;
                        z_desired = h;
                    }
                }
            }
            else if (current_mode == TAKE_PHOTO) {
                save_image = true;
                current_mode = WAITING;
                // ROS_INFO_STREAM("Image saved -> Waiting");
                
            }
            else if (current_mode == APPROACH) {
                current_target_index = idx;
                x_desired = current_drone_pose.pose.position.x + offset[0];
                y_desired = current_drone_pose.pose.position.y + offset[1];
                z_desired = h;
                current_mode = WAITING;
                // ROS_INFO_STREAM("Approaching current target: " << idx);
                
                if (distance(x_desired, y_desired, h, current_drone_pose.pose.position.x, current_drone_pose.pose.position.y, h) < 0.5){
                    current_mode = WAITING;
                    // ROS_INFO_STREAM("Approaching -> waiting: " << idx);
                } 
            }
            else if (current_mode == RETURN) {
                x_desired = initial_pose.pose.position.x;
                y_desired = initial_pose.pose.position.y;
                z_desired = h;
                // ROS_INFO_STREAM("Returning to origin.");
                if (distance(current_drone_pose.pose.position.x, current_drone_pose.pose.position.y, h, initial_pose.pose.position.x, initial_pose.pose.position.y, h) < 0.5) {
                    current_mode = WAITING;
                    // ROS_INFO_STREAM("Returning stuck inside: " << idx);
                }
            } 

            if (gesture == RETURN && current_mode != RETURN) {
                x_desired = initial_pose.pose.position.x; // set the target pose as the initial pose
                y_desired = initial_pose.pose.position.y;
                z_desired = h;
                current_mode = RETURN;
                // ROS_INFO_STREAM("Returning to origin.");
                if (distance(current_drone_pose.pose.position.x, current_drone_pose.pose.position.y, h, initial_pose.pose.position.x, initial_pose.pose.position.y, h) < 0.5){
                    current_mode = WAITING;
                    // ROS_INFO_STREAM("Return -> Waiting: " << idx);
                }
            }
        }
    }



    void positionsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        // cout<<"one position callback"<<endl;
        target_positions_data = *msg;
        if (current_target_index*2 < target_positions_data.data.size()) {
            //warning : positions are stored [x1,y1,x2,y2,x3,y3 ...]
            double fraction = target_positions_data.data[current_target_index*2] - 0.5;
            double FOV = FOV_DEGREES * M_PI / 180.0;
            angle = -fraction * FOV;
            double angle_degrees = -fraction * FOV_DEGREES;
            // ROS_INFO_STREAM("Angle: " << angle_degrees);
            double target_distance = target_person_distance - DISTANCE_OFFSET;
            // ROS_INFO_STREAM("Distance to target: " << target_person_distance);
            offset = create2DRotationMatrix(current_attitude_mission[2]) * createVector(target_distance, angle);
            // ROS_INFO_STREAM("Offset: " << offset);
        } 
    }


    void update() {
        // For the sake of example, let's assume we are calculating target pose as follows:
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose.position.x = x_desired;
        target_pose.pose.position.y = y_desired;
        target_pose.pose.position.z = h;

        // Publish the pose
        target_pose_pub.publish(target_pose);
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber drone_pose_sub;
    ros::Subscriber image_sub;
    ros::Subscriber body_size_sub;
    ros::Subscriber gesture_sub;
    ros::Subscriber position_sub;

    ros::Publisher target_pose_pub;
    // ros::Publisher image_pub;

    geometry_msgs::PoseStamped current_drone_pose;
    sensor_msgs::Image current_drone_image;
    std_msgs::Float64MultiArray body_size_data;
    std_msgs::Int16MultiArray gesture_data;
    std_msgs::Float64MultiArray target_positions_data;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_planner_node_copy");

    MissionPlanner mission_planner;

    ros::Rate loop_rate(10); // Update rate of 10 Hz

    // cout<<"initial"<<endl;
    while (ros::ok()) {
        // ROS_INFO_STREAM("Before update...");
        mission_planner.update();
        // ROS_INFO_STREAM("After update...");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
