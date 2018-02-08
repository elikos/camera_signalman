/**************************************************
 * camera_signalman - Camera_signalman_nodelet.h
 * December 2017
 * Philippe Rivest @ Elikos
**************************************************/

#ifndef PROJECT_CAMERA_SIGNALMAN_NODE_H
#define PROJECT_CAMERA_SIGNALMAN_NODE_H

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <string>
#include <vector>

#include <elikos_msgs/SelectCameraFeedWithFrameID.h>
#include <elikos_msgs/SelectCameraFeedWithIndex.h>
#include <elikos_msgs/SweepCameras.h>

namespace camera_signalman {

    /*!
     * The main class of the camera_signalman nodelet.
     */
    class Camera_signalman_nodelet : public nodelet::Nodelet {

    public:
        /*!
         * Default constructor for the nodelet's main class.
         * Should be empty or minimal; all the heavy lifting is done in the #onInit method.
         * @see Camera_signalman_nodelet::onInit
         */
        Camera_signalman_nodelet() = default;

        /*!
         * Sets the nodelet's listening topic to reroute the latter.
         * If the specified topic is not in the config, the method will return false and no changes will be made.
         * @param topic The topic of the camera feed to reroute
         * @return True if the topic was in the config and was successfully changed
         */
        bool setCurrentCameraSuscriber(const std::string &frameID);

        /*!
         * Sets the nodelet's listening topic to reroute the latter.
         * (Index starts at 0 you Mathlab pricks).
         * If the specified index is out of bounds of the config, the method will return false and no changes will be made.
         * @param cameraIndex The index of the camera feed to reroute
         * @return True if the index isn't out of bounds and was successfully changed
         */
        bool setCurrentCameraSuscriber(int cameraIndex);

        /*!
         * Returns the current subscribed camera feed's index
         * @return the current subscribed camera feed's index
         */
        int getCurrentCameraIndex() const;

    private:

        /*!
         * The private node handle of the nodelet cluster
         */
        ros::NodeHandle nodeHandle_;

        /*!
         * The image transport instance linked to the node handle.
         * A shared_ptr is used because the field has to be initialised in the Camera_signalman_nodelet::onInit method.
         */
        std::shared_ptr<image_transport::ImageTransport> imageTransport_;

//      +--+-----------+--+
//      |  | Publisher |  |
//      +--+-----------+--+
        /*!
         * The topic to publish the rerouted camera feed
         */
        std::string publisher_topic_;

        /*!
         * The queue size of the publisher
         */
        int publisher_queue_size_ = 1;

        /*!
         * The publisher (do I really have to say more?)
         */
        image_transport::Publisher publisher_;


//      +--+-------------+--+
//      |  | Subscribers |  |
//      +--+-------------+--+
        /*!
         * A vector containing the topics specified in the config
         */
        std::vector<std::string> subscribers_camera_feeds_topics_;

        /*!
         * A vector containing the frame ids specified in the config
         */
        std::vector<std::string> subscribers_camera_feeds_frame_ids_;

        /*!
         * A map translating frame ids to topics
         */
        std::map<std::string, std::string> frameIDsAndTopicsMap_;

        /*!
         * The queue size of the subscriber
         */
        int subscribers_camera_feeds_queue_size_ = 1;

        /*!
         * A subscriber to the currently selected camera feed topic
         */
        std::vector<image_transport::Subscriber> cameraSuscribers_;

        /*!
         * The current frame ID to be republished
         */
        std::string currentFrameID_;

//      +--+----------+--+
//      |  | Services |  |
//      +--+----------+--+

        /*!
         * The service server for the selection of the current feed by index.
         */
        ros::ServiceServer selectCameraIndexServiceServer_;

        /*!
         * The service server for the selection of the current feed by frame ID.
         */
        ros::ServiceServer selectCameraFrameIDServiceServer_;

        /*!
         * The service server for executing sweep.
         */
        ros::ServiceServer sweepCamerasServiceServer_;

        /*!
         * The timer ticking every time the cameras has to change
         */
        ros::Timer sweepTimer_;

        /*!
         * The index at which the sweep started
         */
        long sweepStartIndex_;

        /*!
         * The current index of the sweeping process
         */
        long sweepcurrentIndex_;

//      +--+------+--+
//      |  | Init |  |
//      +--+------+--+

        /*!
         * Method called by the nodelet manager to initialise Camera_signalman_nodelet.
         * Shuts down the nodelet in case of initialisation errors.
         */
        void onInit() override;

        /*!
         * Bootstraps the subscribers, the publisher and the service servers
         */
        void init();

        /*!
         * Reads the bundled "ros.yaml" config and configures this instance accordingly
         * @return True if the parameters are all valid
         */
        bool readParameters();

//      +--+-----------+--+
//      |  | Callbacks |  |
//      +--+-----------+--+

        /*!
         * The callback upon reception of a SelectCameraFeedWithIndex service message
         * @param req The request message with the desired feed index
         * @param res The response sent to the client with the result of the callback
         * @return True if the callback was executed successfully
         * @see SelectCameraFeedWithIndex.srv
         */
        bool selectCameraFeedServiceIndexCallback(elikos_msgs::SelectCameraFeedWithIndex::Request &req,
                                                  elikos_msgs::SelectCameraFeedWithIndex::Response &res);

        /*!
         * The callback upon reception of a SelectCameraFeedWithFrameID service message
         * @param req The request message with the desired feed index
         * @param res The response sent to the client with the result of the callback
         * @return True if the callback was executed successfully
         * @see SelectCameraFeedWithFrameID.srv
         */
        bool selectCameraFeedServiceFrameIDCallback(elikos_msgs::SelectCameraFeedWithFrameID::Request &req,
                                                    elikos_msgs::SelectCameraFeedWithFrameID::Response &res);

        /*!
         * The callback upon reception of a SweepCameras service message
         * @param req The request message with the desired time per camera
         * @param res The response sent to the client with the result of the callback
         * @return SweepCameras.srv
         */
        bool sweepCamerasServiceCallback(elikos_msgs::SweepCameras::Request &req,
                                         elikos_msgs::SweepCameras::Response &res);

        /*!
         * The callback executed at each timer tick
         * @param event The timer event associated
         */
        void sweepTimerCallback(const ros::TimerEvent& event);

        /*!
         * The callback executed on reception of an image from the subscribed camera feed.
         * This method simply re-publishes the image on with the #publisher_.
         * @param imageMsg The image sent by the camera feed
         */
        void cameraSuscriberCallback(const sensor_msgs::ImageConstPtr &imageMsg);
    };

}
#endif //PROJECT_CAMERA_SIGNALMAN_NODE_H
