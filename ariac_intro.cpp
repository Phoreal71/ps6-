#include <ros/ros.h>
#include <iostream>
#include <string>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/DroneControl.h>
#include <std_srvs/Trigger.h>

using namespace std;

//Determines if a snapshot should be taken
bool g_take_new_snapshot = false;
//Used to store image date from camera 2
osrf_gear::LogicalCameraImage g_cam2_data;

//Callback function for camera 2 subscriber
void cam2CB(const osrf_gear::LogicalCameraImage& message_holder) {
    if(g_take_new_snapshot) {
        g_cam2_data = message_holder;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ps6");
    ros::NodeHandle n;

    //Services to start up competition, start up conveyor motion, and for drone control
    ros::ServiceClient startup_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    std_srvs::Trigger startup_srv;
    ros::ServiceClient conveyor_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control");
    osrf_gear::ConveyorBeltControl conveyor_srv;
    ros::ServiceClient drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone");
    osrf_gear::DroneControl drone_srv;

    ros::Subscriber cam2_subscriber = n.subscribe("/ariac/logical_camera_2", 1, cam2CB);

    startup_srv.response.success = false;
    while(!startup_srv.response.success) {
        ROS_WARN("not successful starting up yet...");
        startup_client.call(startup_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from startup service");

    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    while(!conveyor_srv.response.success) {
        ROS_WARN("not successful starting conveyor yet...");
        conveyor_client.call(conveyor_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got success response from conveyor service");

    //Loop is exited as soon as a box is seen by camera 2 
    g_take_new_snapshot = true;
    while(g_cam2_data.models.size()<1) {
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("I see a box!");

    //When z position of box0 is -0.3 with respect to camera 2, conveyor is halted for 5 seconds, then movement is resumed
    bool marker = false;
    while(!marker){
      if(g_cam2_data.models[0].pose.position.z < -0.3){
        ROS_INFO("Waiting for Box0 to approach directly under camera 2.....");
        ros::spinOnce();
        ros::Duration(0.5).sleep();
      }
      else{
        ROS_INFO("Box0 is right under logical camera 2, halting conveyor for 5 seconds!");
	conveyor_srv.request.power = 0.0;
	conveyor_srv.response.success = false;
	marker = true;
	conveyor_client.call(conveyor_srv);
	ros::Duration(5.0).sleep();
      }
    }

    ROS_INFO("Resuming conveyor movement...");
    conveyor_srv.request.power = 100.0;
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    //Program waits 15 seconds after conveyor movement is resumed to call drone to pickup box0 at depot
    ros::Duration(15.0).sleep();

    drone_srv.request.shipment_type = "shipping_box_0";
    drone_srv.response.success = false;
    while(!drone_srv.response.success) {
        ROS_WARN("not successful in calling drone yet...");
        drone_client.call(drone_srv);
        ros::Duration(0.5).sleep();
    }
    ROS_INFO("got successful response from drone service");

    //Conveyor movement is stopped and program is ended
    conveyor_srv.request.power = 0.0;
    conveyor_srv.response.success = false;
    conveyor_client.call(conveyor_srv);

    ROS_INFO("End of simulation");
    return 0;
}
