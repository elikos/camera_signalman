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

#include <elikos_msgs/SelectCameraFeedWithTopic.h>
#include <elikos_msgs/SelectCameraFeedWithID.h>

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
        bool selectCameraFeedServiceIDCallback(elikos_msgs::SelectCameraFeedWithID::Request &req,
                                               elikos_msgs::SelectCameraFeedWithID::Response &res);

        bool selectCameraFeedServiceTopicCallback(elikos_msgs::SelectCameraFeedWithTopic::Request &req,
                                                  elikos_msgs::SelectCameraFeedWithTopic::Response &res);

        void cameraSuscriberCallback(const sensor_msgs::Image::Ptr &imageMsg);

        //Private methods

    };

}
#endif //PROJECT_CAMERA_SIGNALMAN_NODE_H
