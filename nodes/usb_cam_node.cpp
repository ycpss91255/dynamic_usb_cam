/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <usb_cam/usb_cam.h>
#include <dynamic_reconfigure/server.h>
#include <usb_cam/usb_cam_paramConfig.h>

#include <sstream>

namespace usb_cam {

class UsbCamNode {
   public:
    // private ROS node handle
    ros::NodeHandle node_;

    // shared image message
    sensor_msgs::Image img_;
    image_transport::CameraPublisher image_pub_;

    // base parameters
    std::string camera_name_, video_device_name_, io_method_name_, pixel_format_name_, camera_info_url_;
    int image_width_, image_height_, framerate_;

    int brightness_, contrast_, saturation_, gain_, sharpness_, backlight_compensation_, white_balance_, autoexposure_, exposure_;
    bool auto_white_balance_;
    bool calibration_mode_ = false;

    boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;

    UsbCam cam_;

    ros::ServiceServer service_start_, service_stop_;


    void setparam(usb_cam::usb_cam_paramConfig &config, uint32_t level){
        ROS_INFO("New imaging param :");
        ROS_INFO("brightness: %d",config.brightness);
        ROS_INFO("contrast: %d",config.contrast);
        ROS_INFO("saturation: %d",config.saturation);
        ROS_INFO("gain: %d",config.gain);
        ROS_INFO("sharpness: %d",config.sharpness);
        ROS_INFO("backlight_compensation: %d",config.backlight_compensation);
        ROS_INFO("auto_white_balance: %s",config.auto_white_balance?"True":"False");
        if(!config.auto_white_balance)
            ROS_INFO("white_balance",config.white_balance);
        ROS_INFO("autoexposure: %d",config.autoexposure);
        ROS_INFO("exposure: %d\n",config.exposure);

        // set camera iamging parameters
        cam_.set_v4l_parameter("brightness", config.brightness);
        cam_.set_v4l_parameter("contrast", config.contrast);
        cam_.set_v4l_parameter("saturation", config.saturation);
        cam_.set_v4l_parameter("gain", config.gain);
        cam_.set_v4l_parameter("sharpness", config.sharpness);
        cam_.set_v4l_parameter("backlight_compensation", config.backlight_compensation);
        // check auto white balance
        if (config.auto_white_balance)
            cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
        else {
            cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
            cam_.set_v4l_parameter("white_balance_temperature", config.autoexposure);
        }
        // check auto exposure
        cam_.set_v4l_parameter("exposure_auto", config.autoexposure);
        cam_.set_v4l_parameter("exposure_absolute", config.exposure);

        }

    bool service_start_cap(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res) {
        cam_.start_capturing();
        return true;
    }

    bool service_stop_cap(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res) {
        cam_.stop_capturing();
        return true;
    }
    UsbCamNode() : node_("~") {
        // advertise the main image topic
        image_transport::ImageTransport it(node_);
        image_pub_ = it.advertiseCamera("image_raw", 1);
        node_.param("calibration_mode",calibration_mode_,false);
        // base parameters
        node_.param("camera_frame_id", img_.header.frame_id,std::string("head_camera"));
        node_.param("camera_name", camera_name_, std::string("head_camera"));
        node_.param("video_device", video_device_name_,std::string("/dev/video0"));
        // possible values: mmap, read, userptr
        node_.param("io_method", io_method_name_, std::string("mmap"));
        // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
        node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
        node_.param("image_width", image_width_, 640);
        node_.param("image_height", image_height_, 480);
        node_.param("framerate", framerate_, 30);

        // camera imaging parameters
        node_.param("brightness", brightness_, 0);                          // min=-64 max=64 step=1 default=0
        node_.param("contrast", contrast_, 32);                             // min=0 max=64 step=1 default=32
        node_.param("saturation", saturation_, 64);                         // min=0 max=128 step=1 default=64
        node_.param("gain", gain_, 0);                                      // min=0 max=100 step=1 default=0
        node_.param("sharpness", sharpness_, 2);                            // min=0 max=6 step=1 default=2
        node_.param("backlight_compensation", backlight_compensation_,1);   // min=0 max=2 step=1 default=1
        // white balance related
        node_.param("auto_white_balance", auto_white_balance_, true);       // default=1
        node_.param("white_balance", white_balance_, 4600);                 // in=2800 max=6500 step=1 default=4600
        // exposure related
        node_.param("autoexposure", autoexposure_, 3);                      // min=0 max=3 default=3 (menu)
        node_.param("exposure", exposure_, 157);                            // min=1 max=5000 step=1 default=157

        // load the external camera info
        node_.param("camera_info_url", camera_info_url_, std::string(""));
        node_.param("a123",12);
        node_.setParam("exposure",90);
        cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));

        // create Services
        service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
        service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

        // check for default camera info
        if (!cinfo_->isCalibrated()) {
            cinfo_->setCameraName(video_device_name_);
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.frame_id = img_.header.frame_id;
            camera_info.width = image_width_;
            camera_info.height = image_height_;
            cinfo_->setCameraInfo(camera_info);
        }

        ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS",
                  camera_name_.c_str(), video_device_name_.c_str(), image_width_,image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

        // set the IO method
        UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
        if (io_method == UsbCam::IO_METHOD_UNKNOWN) {
            ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
            node_.shutdown();
            return;
        }

        // set the pixel format
        UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
        if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN) {
            ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
            node_.shutdown();
            return;
        }

        // start the camera
        cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_, image_height_, framerate_);

        // set camera iamging parameters
        cam_.set_v4l_parameter("brightness", brightness_);
        cam_.set_v4l_parameter("contrast", contrast_);
        cam_.set_v4l_parameter("saturation", saturation_);
        cam_.set_v4l_parameter("gain", gain_);
        cam_.set_v4l_parameter("sharpness", sharpness_);
        cam_.set_v4l_parameter("backlight_compensation", backlight_compensation_);
        // check auto white balance
        if (auto_white_balance_)
            cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
        else {
            cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
            cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
        }
        // check auto exposure
        cam_.set_v4l_parameter("exposure_auto", autoexposure_);
        cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    virtual ~UsbCamNode() { cam_.shutdown(); }

    bool take_and_send_image() {
        // grab the image
        cam_.grab_image(&img_);

        // grab the camera info
        sensor_msgs::CameraInfoPtr ci(
            new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
        ci->header.frame_id = img_.header.frame_id;
        ci->header.stamp = img_.header.stamp;

        // publish the image
        image_pub_.publish(img_, *ci);

        return true;
    }

    bool spin() {
        ros::Rate loop_rate(this->framerate_);
        while (node_.ok()) {
            if (cam_.is_capturing()) {
                if (!take_and_send_image())
                    ROS_WARN("USB camera did not respond in time.");
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        return true;
    }
    dynamic_reconfigure::Server<usb_cam::usb_cam_paramConfig> server;
    dynamic_reconfigure::Server<usb_cam::usb_cam_paramConfig> :: CallbackType CamParamServer;
};

}  // namespace usb_cam

int main(int argc, char **argv) {
    ros::init(argc, argv, "usb_cam");


    usb_cam::UsbCamNode a;
    // create Dynamic Services
    ROS_INFO("%d",a.calibration_mode_);
    if(a.calibration_mode_){
    a.CamParamServer = boost::bind(( &usb_cam::UsbCamNode::setparam), &a, _1, _2);
    a.server.setCallback(a.CamParamServer);
    }

    a.spin();
    return EXIT_SUCCESS;
}


