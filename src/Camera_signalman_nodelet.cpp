//
// Created by technophil98 on 11/16/17.
//

#include <pluginlib/class_list_macros.h>
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
        nodeHandle_.param("/camera_signalman/subscribers/camera_topics",
                          subscribers_camera_feeds_topics_,
                          std::vector<std::string>(0)
        );

        //Their frame id
        nodeHandle_.param("/camera_signalman/subscribers/camera_frame_ids",
                          subscribers_camera_feeds_frame_ids_,
                          std::vector<std::string>(0)
        );

        //Queue size of subscriptions
        nodeHandle_.param("/camera_signalman/subscribers/camera_feeds/queue_size",
                          subscribers_camera_feeds_queue_size_,
                          1
        );

        //***

        if (subscribers_camera_feeds_topics_.empty()) {
            ROS_FATAL("No subscribed feeds are specified in config. Aboding");
            return false;
        }

        //***

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

        //***

        if (publisher_topic_.empty()) {
            ROS_FATAL("The publish topic is not specified in config. Aboding");
            return false;
        }

        return true;
    }


    void Camera_signalman_nodelet::init() {

        imageTransport_ = std::make_shared<image_transport::ImageTransport>(nodeHandle_);

        //Init publisher
        publisher_ = imageTransport_->advertise(publisher_topic_, static_cast<uint32_t>(publisher_queue_size_));

        ROS_INFO("[camera_signalman] Publishing on %s", publisher_topic_.c_str());

        //Matching topics and frame ids
        for (int i = 0; i < subscribers_camera_feeds_topics_.size(); i++){
            auto p = std::make_pair(subscribers_camera_feeds_topics_[i], subscribers_camera_feeds_frame_ids_[i]);
            frameIDsAndTopicsMap_.insert(p);
        }

        //Init subscribers
        for (const std::string &topic: subscribers_camera_feeds_topics_){
            auto subscriber = imageTransport_->subscribe(topic,
                                                         static_cast<uint32_t>(subscribers_camera_feeds_queue_size_),
                                                         &Camera_signalman_nodelet::cameraSuscriberCallback,
                                                         this
            );
            
            ROS_INFO("[camera_signalman] Init subscriber to topic %s", topic.c_str());

            cameraSuscribers_.push_back(subscriber);
        }

        //Setting current frameID to first in list
        currentFrameID_ = subscribers_camera_feeds_frame_ids_[0];

        //Init services

        //select_camera_feed_with_index
        selectCameraIndexServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/select_camera_feed_with_index",
                                                                       &Camera_signalman_nodelet::selectCameraFeedServiceIndexCallback,
                                                                       this);

        //select_camera_feed_with_topic
        selectCameraTopicServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/select_camera_feed_with_topic",
                                                                       &Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback,
                                                                       this);

        sweepTimer_ = nodeHandle_.createTimer(ros::Duration(1.0),
                                              &Camera_signalman_nodelet::sweepTimerCallback,
                                              this);

        //sweep_cameras
        sweepCamerasServiceServer_ = nodeHandle_.advertiseService("/camera_signalman/sweep_cameras",
                                                                   &Camera_signalman_nodelet::sweepCamerasServiceCallback,
                                                                   this);
    }

    void Camera_signalman_nodelet::cameraSuscriberCallback(const sensor_msgs::ImageConstPtr &imageMsg) {

        std::string frameID = imageMsg->header.frame_id;
        std::string cameraTopic = frameIDsAndTopicsMap_[frameID];

        ROS_DEBUG("[camera_signalman] Received image of frame_id %s from topic %s", frameID.c_str(), cameraTopic.c_str());

        if (frameID == currentFrameID_){
            publisher_.publish(imageMsg);

            ROS_DEBUG("[camera_signalman] Re-published image in %s", publisher_topic_.c_str());
        }

    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(int cameraIndex) {
        if (cameraIndex < 0 || cameraIndex >= subscribers_camera_feeds_topics_.size()) return false;

        return setCurrentCameraSuscriber(subscribers_camera_feeds_frame_ids_[cameraIndex]);
    }

    bool Camera_signalman_nodelet::setCurrentCameraSuscriber(const std::string &frameID) {

        //If subscribers_camera_feeds_topics_ contains the desired topic and the current topic is not the desired topic

        bool validTopic =
                frameID != publisher_topic_ &&
                frameID != currentFrameID_ &&
                std::any_of(subscribers_camera_feeds_frame_ids_.begin(),
                            subscribers_camera_feeds_frame_ids_.end(),
                            [frameID](std::string s) { return (s == frameID); }
                );

        if (validTopic) {

            currentFrameID_ = frameID;

            ROS_INFO("Subscribed to %s (%s)", currentFrameID_.c_str(), frameIDsAndTopicsMap_[currentFrameID_].c_str());
        }

        return validTopic;
    }

    bool Camera_signalman_nodelet::selectCameraFeedServiceIndexCallback(
            elikos_msgs::SelectCameraFeedWithIndex::Request &req,
            elikos_msgs::SelectCameraFeedWithIndex::Response &res)
    {
        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_frame_id = currentFrameID_;

        int desiredIndex = req.camera_index;

        setCurrentCameraSuscriber(desiredIndex);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_frame_id = currentFrameID_;

        res.has_changed = (res.old_camera_index != res.new_camera_index) && (res.old_camera_frame_id != res.new_camera_frame_id);

        return true;
    }

    bool Camera_signalman_nodelet::selectCameraFeedServiceTopicCallback(elikos_msgs::SelectCameraFeedWithFrameID::Request &req,
                                                                        elikos_msgs::SelectCameraFeedWithFrameID::Response &res)
    {

        res.old_camera_index = getCurrentCameraIndex();
        res.old_camera_frame_id = currentFrameID_;

        setCurrentCameraSuscriber(currentFrameID_);

        res.new_camera_index = getCurrentCameraIndex();
        res.new_camera_frame_id = currentFrameID_;

        res.has_changed = (res.old_camera_index != res.new_camera_index) && (res.old_camera_frame_id != res.new_camera_frame_id);

        return true;
    }

    int Camera_signalman_nodelet::getCurrentCameraIndex() const {
        return static_cast<int>(std::find(subscribers_camera_feeds_frame_ids_.begin(),
                                          subscribers_camera_feeds_frame_ids_.end(),
                                          currentFrameID_)
                                - subscribers_camera_feeds_frame_ids_.begin()
        );
    }

    bool Camera_signalman_nodelet::sweepCamerasServiceCallback(elikos_msgs::SweepCameras::Request &req,
                                                               elikos_msgs::SweepCameras::Response &res) {
        int msPerCam = (req.ms_per_cam == 0) ? 1000 : req.ms_per_cam;

        if (sweepTimer_.hasPending()) //Do not override sweep
            return false;

        sweepcurrentIndex_ = distance(subscribers_camera_feeds_frame_ids_.begin(),
                                 std::find(subscribers_camera_feeds_frame_ids_.begin(),
                                           subscribers_camera_feeds_frame_ids_.end(),
                                           currentFrameID_)
                                );

        sweepStartIndex_ = sweepcurrentIndex_;

        sweepTimer_.setPeriod(ros::Duration(msPerCam/1000));
        sweepTimer_.start();

        return true;
    }

    void Camera_signalman_nodelet::sweepTimerCallback(const ros::TimerEvent &event) {

        sweepcurrentIndex_ = (sweepcurrentIndex_ + 1) % subscribers_camera_feeds_frame_ids_.size();
        currentFrameID_ = subscribers_camera_feeds_frame_ids_[sweepcurrentIndex_];

        if (sweepcurrentIndex_ == sweepStartIndex_)
            sweepTimer_.stop();
    }
}