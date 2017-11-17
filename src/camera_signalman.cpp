//
// Created by technophil98 on 11/16/17.
//

#include "Camera_signalman_node.h"


int main(int argc, char **argv){


    ros::init(argc,argv, "camera_signalman");

    ros::NodeHandle nodeHandle;

    camera_signalman::Camera_signalman_node cameraSignalmanNode(nodeHandle);

    ros::Rate loop_rate(15);

    ros::spin();

    return 0;
}