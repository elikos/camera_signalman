//
// Created by technophil98 on 11/16/17.
//

#ifndef PROJECT_CAMERA_SIGNALMAN_NODE_H
#define PROJECT_CAMERA_SIGNALMAN_NODE_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

#include "camera_signalman/select_camera_feed_with_ID.h"
#include "camera_signalman/select_camera_feed_with_topic.h"

namespace camera_signalman {

    class Camera_signalman_nodelet : public nodelet::Nodelet {

    public:
        Camera_signalman_nodelet();

        bool setCurrentCameraSuscriber(const std::string &topic);

        bool setCurrentCameraSuscriber(int cameraID);

        int getCurrentCameraIndex() const;

    private:
        ros::NodeHandle nodeHandle_;

        std::string darknet_publisher_topic_;
        int darknet_publisher_queue_size_;
        ros::Publisher darknetPublisher_;

        int subscribers_camera_feeds_queue_size_;
        std::vector<std::string> subscribers_camera_feeds_topics_;
        ros::Subscriber currentCameraSuscriber_;

        ros::ServiceServer selectCameraIDServiceServer_;
        ros::ServiceServer selectCameraTopicServiceServer_;

        //Init
        void onInit() override;
        void init();

        bool readParameters();

        //Callbacks
        bool selectCameraFeedServiceIDCallback(camera_signalman::select_camera_feed_with_ID::Request &req,
                                               camera_signalman::select_camera_feed_with_ID::Response &res);

        bool selectCameraFeedServiceTopicCallback(camera_signalman::select_camera_feed_with_topic::Request &req,
                                                  camera_signalman::select_camera_feed_with_topic::Response &res);

        void cameraSuscriberCallback(const sensor_msgs::Image::ConstPtr &imageMsg);

        //Private methods

    };

}
#endif //PROJECT_CAMERA_SIGNALMAN_NODE_H
