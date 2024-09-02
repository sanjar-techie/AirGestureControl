#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


#define WAITING 99
#define TAKE_PHOTO 1
#define APPROACH 2
#define RETURN 3

#define ONE_METER_AWAY_BODY 0.6
#define DISTANCE_OFFSET 2.0

using namespace ros;
using namespace std;

const double FOV_DEGREES = 69.4;

class MissionPlanner {
public:

    ros::Time last_image_save_time;
    ros::Timer image_save_timer;

    double angle = 0.0;
    double x_desired = 0.0;
    double y_desired = 0.0;
    double z_desired = 0.0;

    int current_mode = WAITING;
    int current_target_index = 0;
    double target_person_distance = 0.0;
    bool save_image = false;

    // Constructor definition
    MissionPlanner() : image_save_timer(nh.createTimer(ros::Duration(10), &MissionPlanner::saveImage, this)) {
        drone_pose_sub = nh.subscribe("mavros/vision_pose/pose", 10, &MissionPlanner::dronePoseCallback, this);
        image_sub = nh.subscribe("/camera/color/image_raw", 10, &MissionPlanner::imageCallback, this);
        neck_to_hip_sub = nh.subscribe("/posenet/neck_to_hip", 10, &MissionPlanner::neckToHip, this);
        neck_pose_sub = nh.subscribe("/posenet/neck_pose", 10, &MissionPlanner::neckPose, this);
        gesture_sub = nh.subscribe("/posenet/gesture_out", 10, &MissionPlanner::gestureCallback, this);

        target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/photo_zone_pose", 10);
        // image_pub = nh.advertise<sensor_msgs::Image>("image_saver_topic", 10);
    }

    void saveImage(const ros::TimerEvent&) {
        if (save_image && (ros::Time::now() - last_image_save_time).toSec() > 10.0) {
            try {
                cv::Mat cv_image = cv_bridge::toCvShare(current_drone_image, "bgr8")->image;
                cv::imwrite("/home/ee478/Pictures/team1.jpg", cv_image);
                last_image_save_time = ros::Time::now();
                ROS_INFO("Image saved.");
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", current_drone_image->encoding.c_str());
            }
        }
    }

    void dronePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (current_mode == WAITING) {
            initial_pose = *msg; // store the initial pose when drone is in WAITING mode
        }
        current_drone_pose = *msg;
    }


    void imageCallback(const sensor_msgs::ImageConstPtr& img) {
        current_drone_image = img;
    }

    void neckToHip(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        // cout<<"one bodysize callback"<<endl;
        neck_to_hip_dist = *msg;
        if (current_target_index < neck_to_hip_dist.data.size()) {
            target_person_distance = ONE_METER_AWAY_BODY / neck_to_hip_dist.data[current_target_index];
        } else {
            // ROS_WARN("Current target index out of bounds in body size data.");
        }
    }

    void neckPose(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        // cout<<"one position callback"<<endl;
        target_positions_data = *msg;
        if (current_target_index*2 < target_positions_data.data.size()) {
            // positions are stored [x1,y1,x2,y2,x3,y3 ...]
            double fraction = target_positions_data.data[current_target_index*2] - 0.5;
            double FOV = FOV_DEGREES * M_PI / 180.0;
            angle = fraction * FOV;
            double target_distance = target_person_distance - DISTANCE_OFFSET;
            x_desired = target_distance * cos(angle);
            y_desired = target_distance * sin(angle);
        } else {
            // ROS_WARN("Current target index out of bounds in target positions data.");
        }
    }

    void gestureCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
        if (msg == NULL) {
            // ROS_WARN("Received NULL Int16MultiArray message");
            return;
        }
        gesture_data = *msg;
        for (int idx = 0; idx < gesture_data.data.size(); idx++) {
            int gesture = gesture_data.data[idx];
            if (gesture == TAKE_PHOTO) {
                if (current_drone_image->data.size() == 0) {
                    ROS_WARN("No current drone image available to save");
                    return;
                }
                save_image = true;
                current_mode = TAKE_PHOTO;
                ROS_INFO_STREAM("Ready to take a photo.");
            } else if ((current_mode == WAITING || current_mode == APPROACH) && gesture == APPROACH) {
                current_target_index = idx;
                current_mode = APPROACH;
                ROS_INFO_STREAM("Approaching current target: " << idx);
            } else if (current_mode == TAKE_PHOTO && gesture == RETURN) {
                x_desired = initial_pose.pose.position.x; // set the target pose as the initial pose
                y_desired = initial_pose.pose.position.y;
                z_desired = initial_pose.pose.position.z;
                current_mode = RETURN;
                ROS_INFO_STREAM("Returning to origin.");
            }
        }
    }

    void update() {
        //let's assume we are calculating target pose as follows:
        geometry_msgs::PoseStamped target_pose;
        target_pose.pose.position.x = x_desired;
        target_pose.pose.position.y = y_desired;
        target_pose.pose.position.z = current_drone_pose.pose.position.z;

        // Publish the pose
        target_pose_pub.publish(target_pose);
    }

private:
    ros::NodeHandle nh;

    ros::Subscriber drone_pose_sub;
    ros::Subscriber image_sub;
    ros::Subscriber neck_to_hip_sub;
    ros::Subscriber gesture_sub;
    ros::Subscriber neck_pose_sub;

    ros::Publisher target_pose_pub;
    // ros::Publisher image_pub;

    geometry_msgs::PoseStamped current_drone_pose;
    std_msgs::Float64MultiArray neck_to_hip_dist;
    std_msgs::Int16MultiArray gesture_data;
    std_msgs::Float64MultiArray target_positions_data;
    geometry_msgs::PoseStamped initial_pose;
    sensor_msgs::ImageConstPtr current_drone_image;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mission_planner_node");

    MissionPlanner mission_planner;

    ros::Rate loop_rate(10); // Update rate of 10 Hz

    // cout<<"initial"<<endl;
    while (ros::ok()) {
        mission_planner.update();
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}
