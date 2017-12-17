//
// Created by technophil98 on 11/16/17.
//

#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include "Camera_signalman_nodelet.h"

PLUGINLIB_EXPORT_CLASS(camera_signalman::Camera_signalman_nodelet, nodelet::Nodelet)


namespace camera_signalman {

    void Camera_signalman_nodelet::onInit() {

        ROS_INFO("Init");

        nodeHandle_ = this->getPrivateNodeHandle();

        // Read parameters from config file.
        if (!readParameters()) {
            ROS_FATAL("Error in config read. Shutting down!");
            ros::requestShutdown();
        }


        init();

        ROS_INFO("Launched");
    }

    bool Camera_signalman_nodelet::readParameters() {

        //Topics to subscribe
        nodeHandle_.param("/camera_signalman/subscribers/camera_feeds/topics",
                          subscribers_camera_feeds_topics_,
                          std::vector<std::string>(0)
        );

        //Queue size of subscriptions
        nodeHandle_.param("/camera_signalman/subscribers/camera_feeds/queue_size",
                          subscribers_camera_feeds_queue_size_,
                          1
        );

        if (subscribers_camera_feeds_topics_.empty()) {
            ROS_FATAL("No subscribed feeds are specified in config. Aboding");
            return false;
        }

        //Queue size of publisher
        nodeHandle_.param("/camera_signalman/publisher/queue_size",
                          publisher_queue_size_,
                          1
        );

        //Publisher topic
        nodeHandle_.param("/camera_signalman/publisher/topic",
                          publisher_topic_,
                          std::string("")
        );


        if (publisher_topic_.empty()) {
            ROS_FATAL("The publish topic is not specified in config. Aboding");
            return false;
        }

        return true;
    }


    void Camera_signalman_nodelet::init() {

        imageTransport_ = std::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(nodeHandle_));

        //Init publisher
        publisher_ = imageTransport_->advertise(publisher_topic_,
                                               static_cast<uint32_t>(publisher_queue_size_)
        );

        ROS_INFO("[camera_signalman_node] Publishing on %s", publisher_topic_.c_str());

        //Init subscriber
        setCurrentCameraSuscriber(0);

        //Init services

        //select_camera_feed_with_index
        selectCameraIndexServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/select_camera_feed_with_index",
                                                                       &Camera_signalman_nodelet::selectCameraFeedServiceIndexCallback,
                                                                       this);

        //select_camera_feed_with_topic
        selectCameraTopicServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/select_camera_feed_with_topic",
                                                                       &Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback,
                                                                       this);
    }

    void Camera_signalman_nodelet::cameraSuscriberCallback(const sensor_msgs::ImageConstPtr &imageMsg) {

        ROS_DEBUG("[camera_signalman_node] Received message at feed %s", currentCameraSuscriber_.getTopic().c_str());

        publisher_.publish(imageMsg);

        ROS_DEBUG("[camera_signalman_node] Re-published message in %s", publisher_topic_.c_str());

    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= subscribers_camera_feeds_topics_.size()) return false;

        return setCurrentCameraSuscriber(subscribers_camera_feeds_topics_[cameraIndex]);
    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(const std::string &topic) {

        //If subscribers_camera_feeds_topics_ contains the desired topic and the current topic is not the desired topic

        bool validTopic =
                topic != publisher_topic_ &&
                topic != currentCameraSuscriber_.getTopic() &&
                std::any_of(subscribers_camera_feeds_topics_.begin(),
                            subscribers_camera_feeds_topics_.end(),
                            [topic](std::string s) { return (s == topic); }
                );

        if (validTopic) {

            //Shutdown old subscriber
            currentCameraSuscriber_.shutdown();
            //Subscribe to new topic
            currentCameraSuscriber_ = imageTransport_->subscribe(topic,
                                                                static_cast<uint32_t>(subscribers_camera_feeds_queue_size_),
                                                                &Camera_signalman_nodelet::cameraSuscriberCallback,
                                                                this
            );

            ROS_INFO("Subscribed to %s", currentCameraSuscriber_.getTopic().c_str());
        }

        return validTopic;
    }

    bool Camera_signalman_nodelet::selectCameraFeedServiceIndexCallback(
            elikos_msgs::SelectCameraFeedWithIndex::Request &req,
            elikos_msgs::SelectCameraFeedWithIndex::Response &res)
    {
        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_topic = currentCameraSuscriber_.getTopic();

        int desiredIndex = req.camera_index;

        setCurrentCameraSuscriber(desiredIndex);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_topic = currentCameraSuscriber_.getTopic();

        res.has_changed = (res.old_camera_index != res.new_camera_index) && (res.old_camera_topic != res.new_camera_topic);

        return true;
    }

    bool Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback(elikos_msgs::SelectCameraFeedWithTopic::Request &req,
                                                                        elikos_msgs::SelectCameraFeedWithTopic::Response &res)
    {

        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_topic = currentCameraSuscriber_.getTopic();

        std::string desiredTopic = req.camera_topic;

        setCurrentCameraSuscriber(desiredTopic);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_topic = currentCameraSuscriber_.getTopic();

        res.has_changed = (res.old_camera_index != res.new_camera_index) && (res.old_camera_topic != res.new_camera_topic);

        return true;
    }

    int Camera_signalman_nodelet::getCurrentCameraIndex() const {
        return static_cast<int>(std::find(subscribers_camera_feeds_topics_.begin(),
                                          subscribers_camera_feeds_topics_.end(),
                                          currentCameraSuscriber_.getTopic())
                                - subscribers_camera_feeds_topics_.begin()
        );
    }
}