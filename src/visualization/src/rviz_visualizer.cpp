#include "ros/ros.h"

#include "msg_types/ControlCmd.h"
#include "msg_types/State.h"
#include "msg_types/StateArray.h"
#include "msg_types/Position.h"
#include "msg_types/PositionArray.h"
#include "msg_types/TrajectoryArray.h"


#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/Point32.h"

#include "std_msgs/Int32.h"
#include "nav_msgs/Path.h"

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf/transform_broadcaster.h>

#include <vector>
#include <cmath>


using namespace std;

// Global Variables:
ros::Publisher marker_pub_objects;
ros::Publisher marker_pub_objects_trajectory;
ros::Publisher marker_pub_planner;
ros::Publisher marker_pub_control;
ros::Publisher marker_pub_state;

// Car message:
geometry_msgs::PolygonStamped robot;
tf::Transform transform_local;
tf::Quaternion q_local;

// Callback Cones Function:

/*
msg_types/orderedCones.msg:

time stamp
geometry_msgs/Point[] Blue
geometry_msgs/Point[] Yellow
carState state

-----------------------------

geometry_msgs/Point.msg:

float64 x
float64 y
float64 z
*/

vector<float> Object_X;
vector<float> Object_Y;

// ID cones
int id_object = 0;
int beta_id_object = id_object;

uint32_t shape = visualization_msgs::Marker::CYLINDER; // Chosen shape 

visualization_msgs::MarkerArray get_marker_objects();


void callback_objects( msg_types::PositionArray array)
{

	for(unsigned int it = 0; it < array.objects.size(); ++it)
	{
        Object_X.push_back(array.objects[it].x);
        Object_Y.push_back(array.objects[it].y);
	}

    // Get MarkerArray message | CONES_SEEN:
    visualization_msgs::MarkerArray msg = get_marker_objects();
    marker_pub_objects.publish(msg);

}

// Get MarkerArray message:
visualization_msgs::MarkerArray get_marker_objects(){

    // Init message:
    visualization_msgs::MarkerArray marker_array;

    // Blue Cones:
    while (!Object_X.empty()) {

        // Create Generic Message:
        // Initialize Message:
        visualization_msgs::Marker marker;
        
        // Frame Reference:
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();

        // Id Cone:
        marker.ns = "object";
        marker.id = id_object++;
        
        marker.type = shape;
        marker.action = visualization_msgs::Marker::ADD;

        marker.pose.position.x = Object_X.back();
        marker.pose.position.y = Object_Y.back();
        marker.pose.position.z = 0.4;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.8;

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration();

        // Pass Condition:
        marker_array.markers.push_back(marker);

        // Empty std::vector(s):
        Object_X.pop_back();
        Object_Y.pop_back();

    }

    id_object = 0;
    return marker_array;
}

/*
objectiu[] objectives
carState state
int32 iden

float32 x
float32 y
float32 heading
float32 curvature
float32 vx
*/

msg_types::StateArray planner;
msg_types::StateArray control;

// Callback Estimation data:
void callback_estimator(msg_types::State msg){

    marker_pub_state.publish(robot);
}


// Callback Planner Function:
void callback_planner(msg_types::StateArray array){

    // Update variable:
    planner = array;

    // Ensure that visualization doesn't die
    if (planner.desired_path.size() <= 2){
        return;
    }

    // Init message:
    nav_msgs::Path marker_planner;

    marker_planner.header.stamp = ros::Time::now();
    marker_planner.header.frame_id = "global";

    for(unsigned int it = 0; it < planner.desired_path.size(); it++ ){

        geometry_msgs::PoseStamped point;

        point.pose.position.x = planner.desired_path[it].x;
        point.pose.position.y = planner.desired_path[it].y;

        marker_planner.poses.push_back(point);
        
    }

    marker_pub_planner.publish(marker_planner);

}


// Callback Control Function:
void callback_control(msg_types::StateArray array_control){

    // Update variable:
    control = array_control;

    // Ensure that visualization doesn't die
    if (control.desired_path.size() <= 2){
        return;
    }

    // Init message:
    nav_msgs::Path marker_control;

    marker_control.header.stamp = ros::Time::now();
    marker_control.header.frame_id = "global";

    for(unsigned int it = 0; it < control.desired_path.size(); it++ ){

        geometry_msgs::PoseStamped point_control;

        point_control.pose.position.x = control.desired_path[it].x;
        point_control.pose.position.y = control.desired_path[it].y;

        marker_control.poses.push_back(point_control);

    }

    marker_pub_control.publish(marker_control);

}

// Main:
int main(int argc, char **argv)
{
    // Init Node:
    ros::init(argc, argv, "rviz_visualizer");
    
    // Handle Connections:
    ros::NodeHandle n;

    // Declare Subscriber and Publisher:
    marker_pub_objects = n.advertise<visualization_msgs::MarkerArray>("/Visualization/objects_detected", 1);
    marker_pub_planner = n.advertise<nav_msgs::Path>("/Visualization/planner", 1);
    marker_pub_control = n.advertise<nav_msgs::Path>("/Visualization/control", 1);
    marker_pub_state = n.advertise<geometry_msgs::PolygonStamped>("/Visualization/loomo", 1);

    ros::Subscriber sub_objects = n.subscribe("/Perception/object_position_array", 1, callback_objects);
    ros::Subscriber sub_planner = n.subscribe("/Path_planning/desired_trajectory", 1, callback_planner);
    ros::Subscriber sub_control = n.subscribe("/Path_planning/length", 1, callback_control);
    ros::Subscriber sub_est_pos = n.subscribe("/pos_info", 1, callback_estimator);    
    
    // NOTE: Make sure that another node is subscribe to the topic to not lose any important data
    while ( marker_pub_objects.getNumSubscribers() < 1 &&
            marker_pub_planner.getNumSubscribers() < 1 &&
            marker_pub_control.getNumSubscribers() < 1 &&
            marker_pub_state.getNumSubscribers() < 1)

    {
        if (!ros::ok()){
            return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    ROS_WARN_ONCE("There is someone subscribed to a visualization topic.\nStarting transmission in '/visualization'.");

    // Create robot polygonShape message:

    float width = 0.5;
    float length = 0.2;

    geometry_msgs::Point32 p1;
    p1.y = width / 2; p1.x = - length / 2;

    geometry_msgs::Point32 p2;
    p2.y = width / 2; p2.x = length / 2;

    geometry_msgs::Point32 p4;
    p4.y = - width / 2; p4.x = length / 2;

    geometry_msgs::Point32 p5;
    p5.y = - width / 2; p5.x = - length / 2;

    geometry_msgs::Point32 p6;
    p6.y = width / 2; p6.x = - length / 2;

    robot.polygon.points.push_back(p1);
    robot.polygon.points.push_back(p2);
    robot.polygon.points.push_back(p4);
    robot.polygon.points.push_back(p5);
    robot.polygon.points.push_back(p6);

    robot.header.stamp = ros::Time::now();
    robot.header.frame_id = "base_link";

    ros::spin();
    return 0;

}