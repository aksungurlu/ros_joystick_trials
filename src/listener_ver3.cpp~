/*

This node listens to the joystick via joy/joy_node node and controls turtlesim/turtlesim_node.
I've used Playstation 3 gamepad for this tutorial application which has 27 axis and 19 buttons.

	L  --> Turns the turtle (publishes vel_cmd message)
	R  --> Moves the turtle (publishes vel_cmd message)
	L2 --> Spawns a new turtle just beside (calls /spawn service)
	R2 --> clears the path (calls /clear service)

First of all, to be able to wrok with joysticks some packages should be installed:

$ sudo apt-get install ros-jade-joystick-drivers

To implement the code for any joystick:
Fİrst check if the joystick driver is loaded by Linux:

	$ ls /dev/input

You should see a similar output to:

	by_id 	event1	event12	event4	event7	js0	mouse1
	by_path	event10	event2	event5	event	mice
	event0	event11	event3	event6	event9	mouse0

The port created is:

	js0

We can check if it's workinusing command jstest as:

	$ sudo jstest /dev/input/js0

Output should show you the status of all axes and buttons 
(as told above for playstation gamepad there are 27 axes -0 to 26- and 19 buttons -0 to 18-):

Axes:	0:  0	1:  0 Buttons:	0:off	1:off

You can check the controls and buttons here and implement the code to use any button - axis to control the turtle.

*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"		//to be able to receive joypad messages
#include "geometry_msgs/Twist.h"	//includes twisting motion coordinates - used by /turtle1/cmd_vel
#include "geometry_msgs/Vector3.h"	//includes 3D point. substructure of Twist.
#include "std_srvs/Empty.h"   		//to be able to call /clear service of turtlesim_node
#include "turtlesim/Spawn.h"		//to be able to call /spawn service of turtlesim_node
#include "turtlesim/Pose.h"		//to be able to interpret pose mesage of turtlesim_node

#include <math.h>			//to be able to use functions sin and cos

ros::Publisher pub;			//  	/turtle1/cmd_vel
ros::ServiceClient client;		// client for /clear service
ros::ServiceClient spawner;		// client for /spawn service
std_srvs::Empty clr;			// arg for calling /clear service
turtlesim::Spawn spw;			// arg for calling /spawn service. includes .request.x/y/theta
bool spw_buf;				//buffer to control spawn button. not to spawn tens of turtles!

//class holding turtle position information

class Pozisyon{				
	
	float x, y, theta;
	public:
	Pozisyon():x(0.0), y(0.0), theta(0.0){
	}
	Pozisyon(float xc, float yc, float thetac):x(xc), y(yc), theta(thetac){
	}
	float* getPoz(){
		float donus[3]={x, y, theta};
		return donus;
	}
	void operator = (Pozisyon p2){
		x = p2.x;
		y = p2.y;
		theta = p2.theta;
	}
	Pozisyon operator + (Pozisyon p2){
		float x_top = x + p2.x;
		float y_top = y + p2.y;
		//float theta_top = theta + p2.theta;
		return Pozisyon(x_top, y_top, theta);
	}
};

Pozisyon poz_dogum;  	//position defined to spawn the new turtle

//subscription for turtlesim_node Pose messages. listened to create the spawn location if needed.

void takipci(const ::turtlesim::Pose& poz){
	float y_norm = -1 * sin(poz.theta);
	float x_norm = -1 * cos(poz.theta);
	poz_dogum = Pozisyon(poz.x, poz.y, poz.theta) + Pozisyon(x_norm, y_norm, poz.theta);
	ROS_INFO("Dogum pozisyonu: %f, %f, %f", *(poz_dogum.getPoz()), *(poz_dogum.getPoz()+1), *(poz_dogum.getPoz()+2));
}

//subscription for joy_node to listen gamepad information.

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
					spw.request.x = *(poz_dogum.getPoz());
					spw.request.y = *(poz_dogum.getPoz()+1);
					spw.request.theta = *(poz_dogum.getPoz()+2);
					spawner.call(spw);
					spw_buf = true;
				}
			}
		else
			spw_buf = false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "listener_ver3");
	ros::NodeHandle n;
	pub = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
	ros::Subscriber sub = n.subscribe("joy", 100, dinleyici);
	ros::Subscriber sub2 = n.subscribe("turtle1/pose", 100, takipci);
	client = n.serviceClient<std_srvs::Empty>("clear");
	spawner = n.serviceClient<turtlesim::Spawn>("spawn");
	spw.request.x = 10.0;
	spw.request.y = 10.0;
	spw.request.theta = 0.0;
	spw_buf=0;	
	ros::spin();
	return 0;
}
