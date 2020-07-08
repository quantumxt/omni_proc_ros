#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core.hpp"
#include "opencv2/ccalib/omnidir.hpp"

class omni_mono_proc {
public:
    omni_mono_proc(const ros::NodeHandle& nh_);
    //Init
    bool init();
    // === CALLBACK & PUBLISHER ===
    void imgCallback(const sensor_msgs::ImageConstPtr& imgp); //Image Input callback

private:
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    image_transport::Publisher imgPub;
    image_transport::Subscriber imgSub;

cv::FileStorage fs;
cv::Mat kMat;
cv::Mat dMat;
cv::Mat xiMat;
cv::Size new_size;

cv::Matx33f Knew;

bool readCalib(const std::string &cfile);
void parseCalib(const cv::FileStorage &fs);
cv::Mat node2array(const cv::FileNode& param, const int& aWidth, const int& aHeight);
};


