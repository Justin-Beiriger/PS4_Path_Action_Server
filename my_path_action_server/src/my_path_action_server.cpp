// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_path_action_server/path_messageAction.h>
#include <example_ros_service/PathSrv.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h> // boolean message
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
//global variables, including publisher and subscriber objects
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 
int g_start = 0; // global var to keep track of how many poses have been completed

class MyPathActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<my_path_action_server::path_messageAction> as_;
    
    // here are some message types to communicate with our client(s)
    my_path_action_server::path_messageGoal goal_; // goal message, received from client
    my_path_action_server::path_messageResult result_; // put results here, to be sent back to the client when done w/ goal
    my_path_action_server::path_messageFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    MyPathActionServer(); //define the body of the constructor outside of class definition

    ~MyPathActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<my_path_action_server::path_messageAction>::GoalConstPtr& goal);
    
    // here are a few useful utility functions:
    double sgn(double x);
    double min_spin(double spin_angle);
    double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
    geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

    void do_halt();
    void do_move(double distance);
    void do_spin(double spin_ang);
    void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

MyPathActionServer::MyPathActionServer() :
   as_(nh_, "path_sequence", boost::bind(&MyPathActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

//signum function: strip off and return the sign of the argument
double MyPathActionServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double MyPathActionServer::min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double MyPathActionServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion MyPathActionServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
void MyPathActionServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//a function to reorient by a specified angle (in radians), then halt
void MyPathActionServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
// this function will be interrupted by a lidar alarm
void MyPathActionServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
    }  
    do_halt(); // halt after movement is complete
}

// calculate yaw and distance to move from start to goal
void MyPathActionServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 
    // store the initial and final (x,y) positions
    double x_start, y_start, x_goal, y_goal;
    
    x_start = current_pose.position.x;
    y_start = current_pose.position.y;
    x_goal = goal_pose.position.x;
    y_goal = goal_pose.position.y;
    
    // store the required travel distance in each direction
    double dx, dy;
    
    dx = x_goal - x_start;
    dy = y_goal - y_start;
    
    dist = sqrt(dx*dx + dy*dy);
    
    if (dist < g_dist_tol)
        heading = convertPlanarQuat2Phi(goal_pose.orientation); 
    else 
        heading = atan2(dy,dx);
}


void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);    
}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void MyPathActionServer::executeCB(const actionlib::SimpleActionServer<my_path_action_server::path_messageAction>::GoalConstPtr& goal) {
    ROS_INFO("in executeCB");
    //do work here: this is where your interesting code goes
    ros::Rate timer(1.0); // 1Hz timer
    
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    geometry_msgs::Pose pose_desired;
    int npts = goal->path.poses.size();
    ROS_INFO("received path request with %d poses",npts);
    
    for (int i=g_start;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = goal->path.poses[i].pose; //get next pose from vector of poses
        
        // get desired heading and distance based on given poses 
        MyPathActionServer::get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,
           pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
        
        // a quaternion is overkill for navigation in a plane; really only need a heading angle
        // this yaw is measured CCW from x-axis
        
        ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);   
        yaw_current = MyPathActionServer::convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = MyPathActionServer::min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        MyPathActionServer::do_spin(spin_angle); // carry out this incremental action
        yaw_current = yaw_current + spin_angle;
        // move forward according to calculated distance
        MyPathActionServer::do_move(travel_distance);
        
        // spin to match the prescribed heading
        spin_angle = convertPlanarQuat2Phi(pose_desired.orientation) - yaw_current;
        MyPathActionServer::do_spin(spin_angle);
	
        // update current pose to remember where we are
        g_current_pose = pose_desired;    
        
        feedback_.path_progress = i;
        as_.publishFeedback(feedback_);
    }
    
    // if we survive to here, then the goal was successfully accomplished; inform the client
    result_.output = npts;
    as_.setSucceeded(result_);
    
}

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "path_action_server_node"); // name this node 
    ros::NodeHandle n;
    do_inits(n); //pass in a node handle so this function can set up publisher with it
    ROS_INFO("instantiating the path_action_server: ");

    MyPathActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ROS_INFO("Ready to accept paths.");
    ros::spin();

    return 0;
}

