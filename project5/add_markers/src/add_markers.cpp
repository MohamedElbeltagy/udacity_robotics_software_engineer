#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

using namespace std;

class addMarkers{
private:
    // Pick-up coordinates
    float pu_x = -2.0;
    float pu_y = 1.0;
    // drop-off coordinates
    float do_x = 0;
    float do_y = 0;

    // Check wether the robot picked-up or dropped off the object
    bool PickedUp = false;
    bool DroppedOff = false;

    // The min distance accuracy of the robot to object 
    float tolerance_value = 1.0;

    float robot_x_pose = 0;
    float robot_y_pose = 0;
    
    float robot_pickup_dis = 0;
    float robot_dropoff_dis = 0;

    ros::NodeHandle n;
    ros::Publisher marker_pub;
    ros::Subscriber odom_sub;

    uint32_t shape = visualization_msgs::Marker::CUBE;
    visualization_msgs::Marker marker;

public:
    // Receive odom data from the robot and calculate it's distance 
    // relative to the pick-up and drop-off markers.
    addMarkers(){
        marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
        odom_sub = n.subscribe("/odom", 1000, &addMarkers::odom_cb, this);

        // Set our initial shape type to be a cube
            
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        
        marker.ns = "add_markers";
        marker.id = 0;

        marker.type = shape;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.1;

        marker.color.r = 1.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.pose.position.x = pu_x;
        marker.pose.position.y = pu_y;
        marker.pose.position.z = 0;
        marker.pose.orientation.w = 1.0;

        marker.lifetime = ros::Duration();

        ROS_INFO("Placing marker at the pick-up zone");

    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        robot_x_pose = msg->pose.pose.position.x;
        robot_y_pose = msg->pose.pose.position.y;

        // If the object not picked nor dropped excecute this code
        if(!PickedUp && !DroppedOff)
        {
            // Calculate the distance from the robot position to the pick-up zone
            robot_pickup_dis = sqrt(pow((pu_x - robot_x_pose), 2) + pow((pu_y - robot_y_pose), 2));
            ROS_INFO("Distance to pick-up zone = %f", robot_pickup_dis);
            if(robot_pickup_dis <= tolerance_value)
            {
                ROS_INFO("Robot reached the pick-up zone");
                PickedUp = true;
            }
        }
        // If the object picked but not dropped excecute this code
        if(PickedUp && !DroppedOff)
        {
            // Calculate the distance from the robot position to the drop-off zone
            robot_dropoff_dis = sqrt(pow((do_x - robot_x_pose), 2) + pow((do_y - robot_y_pose), 2));
            if(robot_dropoff_dis <= tolerance_value)
            {
                ROS_INFO("Arrived at the drop-off zone");
                DroppedOff = true;
                PickedUp = false;
            }
        }
    }

    void pickingUp(){
        marker.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("Object retrieved successfully");
        ros::Duration(3.0).sleep();
    }

    void droppingOff(){
        marker.pose.position.x = do_x;
        marker.pose.position.y = do_y;
        marker.action = visualization_msgs::Marker::ADD;
        ROS_INFO("Object released successfully");
        ros::Duration(3.0).sleep();
    }

    void markerPublisher(){
        marker_pub.publish(marker);
    }

    bool isPicked() {
        return PickedUp;
    }

    bool isDropped() {
        return DroppedOff;
    }
};
 

int main( int argc, char** argv ){
    ros::init(argc, argv, "add_markers");
    addMarkers markers = addMarkers();
    ros::Rate loopRate(1);

    int count = 0;

    
    while (ros::ok()){
        if(markers.isPicked()){
            markers.pickingUp();
        }
        if(markers.isDropped()){
            markers.droppingOff();
        }
        markers.markerPublisher();
    
        ros::spinOnce();
        loopRate.sleep();
    }
    

    // Uncomment the below "while (ros::ok())" block to used with ./add_marker.sh 
    // and comment the above "while (ros::ok())"

    /*
    while (ros::ok()){
        count++;
        if(count == 5){
            markers.pickingUp();
        }
        if(count == 10){
            markers.droppingOff();
        }
        if(count > 10) break;
        markers.markerPublisher();
    
        ros::spinOnce();
        loopRate.sleep();
    }
    */
    return 0;
}