#include <yolov5_pytorch_ros/BoundingBoxesAction.h>
#include <yolov5_pytorch_ros/BoundingBoxesGoal.h>
#include <yolov5_pytorch_ros/BoundingBoxesResult.h>
#include <yolov5_pytorch_ros/BoundingBoxesFeedback.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server

void doneCb(const actionlib::SimpleClientGoalState& state,
        const yolov5_pytorch_ros::BoundingBoxesResultConstPtr& result) {
    std::cout << result->yolo_result << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolov5_action_client_node"); // name this node 
    ros::NodeHandle nh;
    // here is a "goal" object compatible with the server, as defined in example_action_server/action
    yolov5_pytorch_ros::BoundingBoxesGoal goal;

    cv::Mat image = cv::imread("/dataset/Tu_indoor/train/aisle03_img_000006.jpeg");
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    goal.image = *msg;
    ros::Rate rate(1.0);
    // use the name of our server, which is: example_action (named in example_action_server.cpp)
    // the "true" argument says that we want our new client to run as a separate thread (a good idea)
    actionlib::SimpleActionClient<yolov5_pytorch_ros::BoundingBoxesAction> action_client("yolov5_action", true);

    // attempt to connect to the server:
    ROS_INFO("waiting for server: ");
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); // wait for up to 5 seconds
    // something odd in above: does not seem to wait for 5 seconds, but returns rapidly if server not running
    //bool server_exists = action_client.waitForServer(); //wait forever

    if (!server_exists) {
        ROS_WARN("could not connect to server; halting");
        return 0; // bail out; optionally, could print a warning message and retry
    }


    ROS_INFO("connected to action server"); // if here, then we connected to the server;

    while (ros::ok()) {
        action_client.sendGoal(goal, &doneCb); // we could also name additional callback functions here, if desired
        //    action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); //e.g., like this

        bool finished_before_timeout = action_client.waitForResult(ros::Duration(5.0));
        //bool finished_before_timeout = action_client.waitForResult(); // wait forever...
        if (!finished_before_timeout) {
            ROS_WARN("giving up waiting on result");
            return 0;
        } else {
            //if here, then server returned a result to us
	    rate.sleep();
        }

    }

    return 0;
}
