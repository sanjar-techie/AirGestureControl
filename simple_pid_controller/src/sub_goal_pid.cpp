#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <vector>
#include <array>

#define RadToDeg 180/M_PI

using namespace ros;
using namespace std;
bool is_pose_available = false;
double q[4];

void QuaternionToEuler(double& roll, double& pitch, double& yaw)
{	
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
void RegulateVelocity(double& vel, const double limit)
{
	if(abs(vel) > limit)
	{
		vel = vel / abs(vel) * limit;
	}
}

// Class define
class PIDController
{
private:
	NodeHandle nh_;
	NodeHandle nh_param_;
	Publisher pub_;
	Subscriber sub_;
	Subscriber sub_goal_;

	double current_position_[3];
	double current_attitude_[3];
	geometry_msgs::PoseStamped localpose_;

	double goal_[3];

	double Kxy_;
	double Kz_;
	double Kyaw_;

	bool use_yaw_;

	double velxy_limit_;
	double velz_limit_;
	double yaw_limit_;



public:
	// constructor
	PIDController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_param)
	:nh_(nh), nh_param_(nh_param), goal_{0.0,0.0,0.0}, use_yaw_(false)
	{
		pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel",1);
		sub_ = nh_.subscribe("/mavros/vision_pose/pose",1,&PIDController::PoseCallback, this);
		sub_goal_ = nh_.subscribe("/photo_zone_pose",1,&PIDController::GoalCallback, this);


		// Parameter Setting
		if (!nh_param_.getParam("/PIDController/use_yaw",use_yaw_))
        {
            std::cout<<"[Warning] Please set [use_yaw_] parameter, default : false"<<std::endl;
            use_yaw_ = false;
        }
		if (!nh_param_.getParam("/PIDController/Kxy",Kxy_))
        {
            std::cout<<"[Warning] Please set [Kxy] parameter, default : 1.0"<<std::endl;
            Kxy_ = 1.0;
        }
		if (!nh_param_.getParam("/PIDController/Kz",Kz_))
        {
            std::cout<<"[Warning] Please set [Kz] parameter, default : 0.8"<<std::endl;
            Kz_ = 0.8;
        }
		if (!nh_param_.getParam("/PIDController/Kyaw",Kyaw_))
        {
            std::cout<<"[Warning] Please set [Kyaw] parameter, default : 3.0"<<std::endl;
            Kyaw_ = 3.0;
        }
		if (!nh_param_.getParam("/PIDController/velxy_limit",velxy_limit_))
        {
            std::cout<<"[Warning] Please set [velxy_limit] parameter, default : 0.4"<<std::endl;
            velxy_limit_ = 0.4;
        }
		if (!nh_param_.getParam("/PIDController/velz_limit",velz_limit_))
        {
            std::cout<<"[Warning] Please set [velz_limit] parameter, default : 0.2"<<std::endl;
            velz_limit_ = 0.2;
        }
		if (!nh_param_.getParam("/PIDController/yaw_limit",yaw_limit_))
        {
            std::cout<<"[Warning] Please set [yaw_limit] parameter, default : 0.15"<<std::endl;
            yaw_limit_ = 0.15;
        }
		std::cout<<"================================="<<std::endl;
		std::cout<<"Parameters"<<std::endl
		<<"use_yaw_ : "<< (use_yaw_ ? "true" : "false")<<endl
		<<"Kxy_     : "<<Kxy_<<endl
		<<"Kz_      : "<<Kz_<<endl
		<<"Kyaw_    : "<<Kyaw_<<endl;
		std::cout<<"================================="<<std::endl;
	}
	void PublishVelocity()
	{



		//calculate linear velocity
		double cmd_x = (goal_[0] - current_position_[0]) * Kxy_;
		double cmd_y = (goal_[1] - current_position_[1]) * Kxy_;
		double cmd_z = (goal_[2] - current_position_[2]) * Kz_;
		RegulateVelocity(cmd_x,velxy_limit_);
		RegulateVelocity(cmd_y,velxy_limit_);
		RegulateVelocity(cmd_z,velz_limit_);

		// current yaw angle
		double current_yaw = current_attitude_[2];
		double goal_yaw = std::atan2(goal_[1]-current_position_[1],goal_[0]-current_position_[0]);
		// double goal_yaw = 0; 
		double yaw_diff = goal_yaw - current_yaw;

		// regulate yaw angle
		while(abs(yaw_diff)>M_PI)
		{
			if(yaw_diff>0)
			{	yaw_diff = yaw_diff - 2 * M_PI;	}
			else if(yaw_diff<0)
			{	yaw_diff = yaw_diff + 2 * M_PI;	}
		}

		
		double cmd_r = yaw_diff * Kyaw_;
		RegulateVelocity(cmd_r,yaw_limit_);
		
		geometry_msgs::TwistStamped command_msg;
		command_msg.header.stamp = ros::Time::now();


		if ( current_position_[2] < 0.3)
		{   cmd_r = 0;	}

		// assign
		command_msg.twist.linear.x = cmd_x;
		command_msg.twist.linear.y = cmd_y; 
		command_msg.twist.linear.z = cmd_z; 

		//command_msg.twist.angular.z = 0; 
		command_msg.twist.angular.z = cmd_r; 
		pub_.publish(command_msg);
	}
	void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
	{
		localpose_= *msg;

		current_position_[0] = localpose_.pose.position.x;	
		current_position_[1] = localpose_.pose.position.y;
		current_position_[2] = localpose_.pose.position.z;

		q[0] = localpose_.pose.orientation.x; 
		q[1] = localpose_.pose.orientation.y; 
		q[2] = localpose_.pose.orientation.z; 
		q[3] = localpose_.pose.orientation.w; 

		QuaternionToEuler(current_attitude_[0],current_attitude_[1],current_attitude_[2]);
		is_pose_available = true;
	}
	void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
	{
		goal_[0] = msg->pose.position.x;
		goal_[1] = msg->pose.position.y;
		goal_[2] = msg->pose.position.z;
	}
	double GetDistToGoal()
	{
		return sqrt(pow(goal_[0] - current_position_[0] , 2)
			+ pow(goal_[1] - current_position_[1] , 2)
			+ pow(goal_[2] - current_position_[2] , 2));
	}
	// add xy dist function
	double GetXYDistToGoal()
	{
		return sqrt(pow(goal_[0] - current_position_[0] , 2)
			+ pow(goal_[1] - current_position_[1] , 2));
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "PIDController");
	ros::NodeHandle nh;
  	ros::NodeHandle nh_param("~");
	PIDController controller(nh, nh_param);

	ros::Rate loop_rate(30);

	cout<<"start sub_Goal_PID controller"<<endl;

	// main loop
	static int count = 0;
	while(ok())
	{
		controller.PublishVelocity();

		loop_rate.sleep();
		spinOnce();

		if(count%10==0)
		{
			// cout<<"==================="<<endl;
			// cout<<"print count : "<<count<<endl;
			// cout<<"sub goal"<<std::endl;
		}
		count++;
	}
	return 0;
}
