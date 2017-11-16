//
// Created by technophil98 on 11/16/17.
//

#include "Camera_signalman_node.h"
#include "ros/ros.h"



int main(int argc, char **argv){


    ros::init(argc,argv, "camera_signalman");

    ros::NodeHandle nodeHandle;

    Camera_signalman_node cameraSignalmanNode(nodeHandle);

    ros::Rate loop_rate(15);

    ros::spin();

    return 0;
}