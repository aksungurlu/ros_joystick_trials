#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "std_srvs/Empty.h"
#include "turtlesim/Spawn.h"

/*float ax1 = 0.0;
float ax2 = 0.0;
float ax3 = 0.0;
float ax4 = 0.0;*/

ros::Publisher pub;
ros::ServiceClient client;
ros::ServiceClient spawner;
std_srvs::Empty clr;
turtlesim::Spawn spw;
bool spw_buf;

void dinleyici(const sensor_msgs::Joy::ConstPtr& joyc){
	/*float ax1 = joyc->axes[0];
	float ax2 = joyc->axes[1];
	float ax3 = joyc->axes[2];
	float ax4 = joyc->axes[3];*/
	if(joyc->axes[1] != 0.0 || joyc->axes[2] != 0.0 || joyc->axes[3] != 0.0 || joyc->buttons[9] != 0 || joyc->buttons[8] != 0){
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
		ROS_INFO("Axes - linear_x: %f, linear_y: %f, angular: %f", joyc->axes[3], joyc->axes[2], joyc->axes[1]);
		ROS_INFO("Buttons pressed - clear: %d spawn: %d", joyc->buttons[9], joyc->buttons[8]);

		if(joyc->buttons[9] == 1){
			client.call(clr);
			}
		if(joyc->buttons[8] == 1){
				if(!spw_buf){
					spawner.call(spw);
					spw_buf = true;
				}
			}
		else
			spw_buf = false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener_ver1");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	ros::Subscriber sub = n.subscribe("joy", 100, dinleyici);
	client = n.serviceClient<std_srvs::Empty>("clear");
	spawner = n.serviceClient<turtlesim::Spawn>("spawn");
	spw.request.x = 10.0;
	spw.request.y = 10.0;
	spw.request.theta = 0.0;
	spw_buf=0;
	ros::spin();
	return 0;
}
