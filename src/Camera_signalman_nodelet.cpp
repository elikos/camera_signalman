//
// Created by technophil98 on 11/16/17.
//

#include <pluginlib/class_list_macros.h>
#include <Camera_signalman_nodelet.h>

PLUGINLIB_EXPORT_CLASS(camera_signalman::Camera_signalman_nodelet, nodelet::Nodelet)

#include <algorithm>
#include "Camera_signalman_nodelet.h"

namespace camera_signalman {


    Camera_signalman_nodelet::Camera_signalman_nodelet() = default;


    void Camera_signalman_nodelet::onInit() {
        ROS_INFO("[camera_signalman_node] Init");

        nodeHandle_ = this->getPrivateNodeHandle();

        // Read parameters from config file.
        if (!readParameters()) {
            ROS_FATAL("Error in config read. Shutting down!");
            ros::requestShutdown();
        }


        init();

        ROS_INFO("[camera_signalman_node] Launched");
    }

    bool Camera_signalman_nodelet::readParameters() {

        nodeHandle_.param("/camera_signalman/subscribers/camera_feeds/topics", subscribers_camera_feeds_topics_,
                          std::vector<std::string>(0));
        nodeHandle_.param("/camera_signalman/subscribers/camera_feeds/queue_size", subscribers_camera_feeds_queue_size_,
                          1);

        if (subscribers_camera_feeds_topics_.empty()) {
            ROS_FATAL("No subscribed feeds are specified in config. Aboding");
            return false;
        }

        nodeHandle_.param("/camera_signalman/publishers/darknet_publisher/queue_size", darknet_publisher_queue_size_,
                          1);
        nodeHandle_.param("/camera_signalman/publishers/darknet_publisher/topic", darknet_publisher_topic_,
                          std::string(""));


        if (darknet_publisher_topic_.empty()) {
            ROS_FATAL("The publish topic is not specified in config. Aboding");
            return false;
        }

        return true;
    }


    void Camera_signalman_nodelet::init() {

        darknetPublisher_ = nodeHandle_.advertise<sensor_msgs::Image>(darknet_publisher_topic_,
                                                                      static_cast<uint32_t>(darknet_publisher_queue_size_)
        );

        ROS_INFO("[camera_signalman_node] Publishing on %s", darknet_publisher_topic_.c_str());

        setCurrentCameraSuscriber(0);

        selectCameraIDServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/select_camera_feed_with_ID",
                                                                    &Camera_signalman_nodelet::selectCameraFeedServiceIDCallback,
                                                                    this);
        selectCameraTopicServiceServer_ = nodeHandle_.advertiseService(
                "/camera_signalman/select_camera_feed_with_topic",
                &Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback, this);
    }

    void Camera_signalman_nodelet::cameraSuscriberCallback(const sensor_msgs::Image::ConstPtr &imageMsg) {

        ROS_DEBUG("[camera_signalman_node] Received message at feed %s", currentCameraSuscriber_.getTopic().c_str());

        darknetPublisher_.publish(imageMsg);

        ROS_DEBUG("[camera_signalman_node] Re-published message in %s", darknet_publisher_topic_.c_str());

    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(int cameraID) {
        if (cameraID < 0 || cameraID >= subscribers_camera_feeds_topics_.size()) return false;
        return setCurrentCameraSuscriber(subscribers_camera_feeds_topics_[cameraID]);
    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(const std::string &topic) {

        //If subscribers_camera_feeds_topics_ contains the desired topic and the current topic is not the desired topic
        if (std::any_of(subscribers_camera_feeds_topics_.begin(),
                        subscribers_camera_feeds_topics_.end(),
                        [topic](std::string s) { return (s == topic); }
        )

            &&

            topic != darknet_publisher_topic_

            &&

            topic != currentCameraSuscriber_.getTopic()
                ) {

            //Shutdown old subscriber
            currentCameraSuscriber_.shutdown();
            //Subscribe to new topic
            currentCameraSuscriber_ = nodeHandle_.subscribe(topic,
                                                            static_cast<uint32_t>(subscribers_camera_feeds_queue_size_),
                                                            &Camera_signalman_nodelet::cameraSuscriberCallback,
                                                            this
            );

            ROS_INFO("[camera_signalman_node] Subscribed to %s", currentCameraSuscriber_.getTopic().c_str());

            return true;

        } else {

            return false;

        }

    }

    bool
    Camera_signalman_nodelet::selectCameraFeedServiceIDCallback(camera_signalman::select_camera_feed_with_ID::Request &req,
                                                             camera_signalman::select_camera_feed_with_ID::Response &res) {

        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_topic = currentCameraSuscriber_.getTopic();

        int desiredIndex = req.camera_index;
        bool success = false;

        if (desiredIndex >= 0 && desiredIndex < subscribers_camera_feeds_topics_.size())
            success = setCurrentCameraSuscriber(desiredIndex);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_topic = currentCameraSuscriber_.getTopic();

        return success;
    }

    bool Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback(
            camera_signalman::select_camera_feed_with_topic::Request &req,
            camera_signalman::select_camera_feed_with_topic::Response &res) {

        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_topic = currentCameraSuscriber_.getTopic();

        std::string desiredTopic = req.camera_topic;
        bool success = false;

        if (!desiredTopic.empty()) {}
        success = setCurrentCameraSuscriber(desiredTopic);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_topic = currentCameraSuscriber_.getTopic();

        return success;
    }

    int Camera_signalman_nodelet::getCurrentCameraIndex() const {
        return static_cast<int>(std::find(subscribers_camera_feeds_topics_.begin(),
                                          subscribers_camera_feeds_topics_.end(),
                                          currentCameraSuscriber_.getTopic())
                                - subscribers_camera_feeds_topics_.begin()
        );
    }
}