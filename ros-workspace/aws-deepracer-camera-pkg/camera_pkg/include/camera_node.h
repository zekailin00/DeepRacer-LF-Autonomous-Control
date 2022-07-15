///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "deepracer_interfaces_pkg/srv/video_state_srv.hpp"
#include "deepracer_interfaces_pkg/msg/camera_msg.hpp"
#include "opencv2/opencv.hpp"

#include <thread>
#include <atomic>
#include <memory>

#define DEFAULT_IMAGE_WIDTH 160
#define DEFAULT_IMAGE_HEIGHT 120


class CameraNode : public rclcpp::Node {
    /// This class creates the camera_node responsible to read data from the cameras
    /// and publish it as images.
    public:
        const char* CAMERA_MSG_TOPIC = "video_mjpeg";
        const char* DISPLAY_MSG_TOPIC = "display_mjpeg";
        const char* ACTIVATE_CAMERA_SERVICE_NAME = "media_state";

        /// @param node_name Name of the node to be created.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        CameraNode(const std::string & node_name, const std::vector<int> cameraIdxList);

        ~CameraNode() = default;

        /// Scan camera index to add valid streamers to Video Capture List.
        /// @param cameraIdxList List of camera indexes to iterate over to find the valid
        ///                      indices where the camera is connected.
        void scanCameraIndex(const std::vector<int> cameraIdxList);
        /// Request handler for the media server.
        void videoProducerStateHdl(const std::shared_ptr<rmw_request_id_t> request_header,
                                   const std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Request> req,
                                   std::shared_ptr<deepracer_interfaces_pkg::srv::VideoStateSrv::Response> res);

        /// Collects the frame from the camera and publishes them. This method is intended
        /// to run in a separate thread.
        void produceFrames();

        /// ROS publisher object to the publish camera images to camera message topic.
        rclcpp::Publisher<deepracer_interfaces_pkg::msg::CameraMsg>::SharedPtr videoPub_;
        /// ROS publisher object to the publish camera images to display message topic.
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr displayPub_;
        /// ROS service object to activate the camera to publish images.
        rclcpp::Service<deepracer_interfaces_pkg::srv::VideoStateSrv>::SharedPtr activateCameraService_;
        /// Boolean for starting and stopping the worker thread.
        std::atomic<bool> produceFrames_;
        /// Boolean to resize images.
        std::atomic<bool> resizeImages_;
        /// List of OpenCV video capture object used to retrieve frames from the cameras.
        std::vector<cv::VideoCapture> videoCaptureList_;
        /// List of valid camera indices identified after scanning.
        std::vector<int> videoIndexList_;
        /// Thread that constantly retrieves frames from the camera and publishes the frames.
        std::thread videoWorker_;
        /// Camera index parameter to capture video frames from the specific camera.
        int cameraIndex_;
    };



// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     //! TODO Remove hardcode after testing.
//     // TODO Update not to log uvcvideo: Buffer is NULL in /var/log/syslog

//     // Earlier logic of having Left/Center/Right map to index 4/1/0 doesn't work in Ubuntu 20.04
//     // The camera indexes keep changing intermittently in Ubuntu 20.04.
//     // Hence modifying the logic to scan for indexes in descending order and add to the Video Capture list of the valid capture elements.
//     // In case of Stereo Cameras: The index with greater number represents Left Camera.
//     std::vector<int> cameraIndex {4, 3, 2, 1, 0};
//     // Create the camera_node.
//     rclcpp::spin(std::make_shared<MediaEng::CameraNode>("camera_node", cameraIndex));
//     rclcpp::shutdown();
//     return 0;
// }
//!
//*
//?
//todo
//TODO:
//FIXME:
/**
 * @brief 
 * 
 */
