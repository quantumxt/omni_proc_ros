#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>

#ifndef CV_HEADER
#define CV_HEADER
#include "opencv2/core.hpp"
#endif
#include "opencv2/ccalib/omnidir.hpp"

#include "omni_proc/mono_paramConfig.h"

class omni_mono_proc {
public:
    omni_mono_proc(const ros::NodeHandle& nh_);
    //Init
    bool init();
    // === CALLBACK & PUBLISHER ===
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp); //Image Input callback
    void dr_callback(const omni_proc::mono_paramConfig& config, const uint32_t& level);

private:
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    image_transport::Publisher imgPub;
    image_transport::Subscriber imgSub;
    //Dynamic reconfig
    dynamic_reconfigure::Server<omni_proc::mono_paramConfig> mp_server;
    dynamic_reconfigure::Server<omni_proc::mono_paramConfig>::CallbackType mp_cb;

    cv::Mat kMat;
    cv::Mat dMat;
    cv::Mat xiMat;
    cv::Size new_size;

    float zoomOut{ 5 }; //Best around 2-6, negative would flip image
    int flags_out = cv::omnidir::RECTIFY_PERSPECTIVE;
    float aspectRatio{ 1.7 };

    cv::Matx33f Knew;

};

