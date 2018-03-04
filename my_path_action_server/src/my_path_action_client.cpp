// timer_client: works together with action server called "timer_action"
// in source: example_action_server_w_fdbk.cpp
// this code could be written using classes instead (e.g. like the corresponding server)
//  see: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <my_path_action_server/path_messageAction.h>
#include <example_ros_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <std_msgs/Bool.h> // boolean message
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;
bool g_lidar_alarm = false;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

void alarmCallBack(const std_msgs::Bool& alarm_msg)
{
	g_lidar_alarm = alarm_msg.data; // make the alarm status global
	if (g_lidar_alarm) {
		ROS_INFO("LIDAR alarm received!");
	}
}


// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const my_path_action_server::path_messageResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    //ROS_INFO("got result output = %d",result->output);
    //g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const my_path_action_server::path_messageFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->path_progress);
    g_fdbk = fdbk_msg->path_progress; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "timer_client_node"); // name this node 
        ros::NodeHandle n;
        
        // declare the subscriber to listen for the lidar alarm 
        ros::Subscriber alarm_subscriber = n.subscribe("lidar_alarm", 1, alarmCallBack); 
        
        ros::Rate main_timer(1.0);
        // here is a "goal" object compatible with the server, as defined in example_action_server/action
        my_path_action_server::path_messageGoal goal; 
        
        // use the name of our server, which is: timer_action (named in example_action_server_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<my_path_action_server::path_messageAction> action_client("path_sequence", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        
        //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        pose.position.x = 3.0; // first desired x-coord is 3
        pose.position.y = 0.0;
        pose.position.z = 0.0; // let's hope so!
        geometry_msgs::Quaternion quat;
        quat = convertPlanarPhi2Quaternion(1.571);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose = pose;
        goal.path.poses.push_back(pose_stamped);

        // some more poses...
        quat = convertPlanarPhi2Quaternion(0); // get a quaternion corresponding to this heading
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.y = 3.0; // desired y-coord is 3.0
        goal.path.poses.push_back(pose_stamped);

        quat = convertPlanarPhi2Quaternion(1.571);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.x = 6.5; // desired x-coord is 6.5
        goal.path.poses.push_back(pose_stamped);

        quat = convertPlanarPhi2Quaternion(3.14159);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.y = 5.2; // desired x-coord is 6.5
        goal.path.poses.push_back(pose_stamped);

        quat = convertPlanarPhi2Quaternion(1.571);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.x = 3.0; // desired x-coord is 6.5
        goal.path.poses.push_back(pose_stamped);

        quat = convertPlanarPhi2Quaternion(3.14159);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.y = 12.0; // desired x-coord is 6.5
        goal.path.poses.push_back(pose_stamped);

        quat = convertPlanarPhi2Quaternion(3.14159);
        pose_stamped.pose.orientation = quat;
        pose_stamped.pose.position.x = 0; // desired x-coord is 6.5
        goal.path.poses.push_back(pose_stamped);
        
        action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
        
        
    return 0;
}

