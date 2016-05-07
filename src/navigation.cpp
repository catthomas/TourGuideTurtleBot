#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

enum CurrentAction
{
	CRUISE,
	HALT,
	PLAN,
	TOUR
};

/* Variables for movement */
geometry_msgs::Twist twist;
ros::Publisher halt_publisher;

/* Helper Variables */
uint8_t bumper;
CurrentAction action = PLAN;
int tourStop;

/* Locations */
geometry_msgs::PoseStamped computer_lab;
geometry_msgs::PoseStamped west_doors;
geometry_msgs::PoseStamped east_doors;
geometry_msgs::PoseStamped ddl;
geometry_msgs::PoseStamped board_room;
geometry_msgs::PoseStamped tour_flag;

/* Methods */
void publishHalt();
void publishPlan(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *action_client, geometry_msgs::PoseStamped goal);

// define the callback functions for topics we'll subscribe to
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &bump_in);

// create a valid PoseStamped object
void definePoseStamped(geometry_msgs::PoseStamped* poseStamped, float x, float y);

// verify that user input is valid
geometry_msgs::PoseStamped checkInput(geometry_msgs::PoseStamped pose_stamped, int input);

// provide information along the tour
void dispenseInfo(int tourStop);

/* MAIN: START ALL PROCESSES */
int main(int argc, char **argv) {
	ros::init(argc, argv, "turtlebot_navigation");
	ros::NodeHandle n;
	ros::Rate loop_rate(15);

	/* Subscribers (Sensors) */
	ros::Subscriber bumper_subscriber = n.subscribe("mobile_base/events/bumper", 1000, bumperCallback);

	/* Publishers (Commands) */
	halt_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/halt", 1000);

	/* Define locations */
	definePoseStamped(&computer_lab, 0.5, 0.0);
	definePoseStamped(&west_doors, -1.0, -1.0);
	definePoseStamped(&east_doors, 1.0, 0.5);
	definePoseStamped(&ddl, 0.0, 0.0);
	definePoseStamped(&board_room, -0.5, -0.5);

	// an invalid goal that can be used to mark the start of a tour.
	definePoseStamped(&tour_flag, 0.0, -99.0);

	/* action client for sending goals */
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client("move_base", true);

	/* wait for the action server to come up */
	while(!action_client.waitForServer(ros::Duration(5.0))){}

		while (ros::ok())
		{
			if (action == HALT)
			{
				// Robot bumped into something
				publishHalt();
				ros::Duration(5.0).sleep();
				action = CRUISE;
			}
			else if (action == PLAN)
			{
				geometry_msgs::PoseStamped goal;

				// dummy goal value - when it changes, we know we have a valid location
				goal.pose.position.x = 999;
				while (goal.pose.position.x == 999)
				{
					// Ask for input
					cout <<"Choose goal location:" << endl;
					cout << "1) West Doors" << endl;
					cout << "2) Board Room" << endl;
					cout << "3) Digital Design Lab" << endl;
					cout << "4) Computer Lab" << endl;
					cout << "5) East Doors" << endl;
					cout << "6) Full Tour" << endl;
					cout << "0) Quit" << endl;

					int input;
					cin >> input;

					if (input == 0)
					{
						cout << "Exiting..." << endl;
						return 0;
					}

					// Validate input
					goal = checkInput(goal, input);
				}

			// check if the user wants a tour
			if (goal.pose.position.y == tour_flag.pose.position.y)
			{
				// begin tour by going to west doors
				publishPlan(&action_client, west_doors);
				tourStop = 0;
				action = TOUR;		
			}
			else
			{
				// Single Location
				// Plan
				publishPlan(&action_client, goal);

			     // Reset action
				action = CRUISE;
			}
		}
		else if (action == TOUR)
		{
			// current stop is complete	
			if (action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				// pause for info, then move on to the next stop
				tourStop++;
				dispenseInfo(tourStop);
				switch(tourStop)
				{
					case 1:
					publishPlan(&action_client, board_room);
					break;
					case 2:
					publishPlan(&action_client, ddl);
					break;
					case 3:
					publishPlan(&action_client, computer_lab);
					break;
					case 4:
					publishPlan(&action_client, east_doors);
					break;
					case 5:
					action = PLAN;
					break;
				}
			}
		}
		else if (action_client.getState().isDone())
		{
		    // if the current action is finished, plan again
			action = PLAN;
		}

		// Sleep for the specified loop rate before looping again
		loop_rate.sleep();

		// Spin
		ros::spinOnce();
	}

return 0;
}

/* halt */
void publishHalt() {
	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	halt_publisher.publish(twist);
}

void publishPlan(actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *action_client, geometry_msgs::PoseStamped goal) {
	// Define where to go
	goal.header.stamp = ros::Time::now();

	move_base_msgs::MoveBaseGoal wrapper;
	wrapper.target_pose = goal;

	action_client->sendGoal(wrapper);
}

//Fetch the sensor state
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &bump_in) {
	ROS_INFO("bumper hit! value = [%d]", bump_in->bumper);

	bumper = bump_in->bumper;
	if (bumper != 0)
	{
		action = HALT;
	}
}

void definePoseStamped(geometry_msgs::PoseStamped* poseStamped, float x, float y) {
	poseStamped->header.frame_id = "map";
	poseStamped->pose.position.x = x;
	poseStamped->pose.position.y = y;
	poseStamped->pose.position.z = 0.0;
	poseStamped->pose.orientation.x = 0.0;
	poseStamped->pose.orientation.y = 0.0;
	poseStamped->pose.orientation.z = 0.0;
	poseStamped->pose.orientation.w = 1.0;
}

geometry_msgs::PoseStamped checkInput(geometry_msgs::PoseStamped pose_stamped, int input) {
	switch(input) {
		case 1: return west_doors;
		case 2: return board_room;
		case 3: return ddl;
		case 4: return computer_lab;
		case 5: return east_doors;
		case 6: return tour_flag;
		default: return pose_stamped;
	}
}

void dispenseInfo(int tourStop) {
	switch(tourStop)
	{
		case 1:
		cout << "\nThis is Devon Energy Hall. We're at the west doors. Construction was completed in 2010 and it houses our Computer Science and Electrical and Computer Engineering Programs.\n" << endl;
		break;
		case 2:
		cout << "This is the Boardroom. It is used for high-level presentations from visitors, faculty, and is used for thesis and dissertation defenses.\n" << endl;
		break;
		case 3:
		cout << "This is the Digital Design Lab, which is the lab for senior level Electrical and Computer Engineering students. In this lab, students work on advanced projects at the intersection of hardware and software, designing circuits and writing programs that interact with one another, such as digital thermometers, radios, games like Guitar Hero, and home automation systems.\n" << endl;
		break;
		case 4:
		cout << "This is the Computer Lab. It's reserved exlusively for Computer Science students, and contains some fairly powerful computers that have specialized software installed on them. There are Windows, Mac, and Ubuntu computers in the lab, so students have the resources they need no matter what they're working on.\n" << endl;
		break;
		case 5:
		cout << "Tour Complete! Thanks for joining me!\n" << endl;
		break;
	}
	ros::Duration(5.0).sleep();
}
