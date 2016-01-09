#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"

/*float ax1 = 0.0;
float ax2 = 0.0;
float ax3 = 0.0;
float ax4 = 0.0;*/

ros::Publisher pub;

void dinleyici(const sensor_msgs::Joy::ConstPtr& joyc){
	/*float ax1 = joyc->axes[0];
	float ax2 = joyc->axes[1];
	float ax3 = joyc->axes[2];
	float ax4 = joyc->axes[3];*/
	if(joyc->axes[1] != 0.0 || joyc->axes[2] != 0.0 || joyc->axes[3] != 0.0 || joyc->buttons[9] != 0){
		geometry_msgs::Vector3 angular;
		angular.z = joyc->axes[1];
		//angular.y = joyc->axes[1];
		geometry_msgs::Vector3 linear;
		linear.x = joyc->axes[3];
		linear.y = joyc->axes[2];
		geometry_msgs::Twist twistegel;
		twistegel.angular = angular;
		twistegel.linear = linear;
		pub.publish(twistegel);
		ROS_INFO("I heard axis1: %f, axis2: %f, axis3: %f, axis4: %f button9: %d", joyc->axes[0], joyc->axes[1], joyc->axes[2], joyc->axes[3], joyc->buttons[9]);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener_ver1");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	ros::Subscriber sub = n.subscribe("joy", 100, dinleyici);
	ros::spin();
	return 0;
}
