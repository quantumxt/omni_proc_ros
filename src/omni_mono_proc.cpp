#include "omni_proc/omni_mono_proc.hpp"
#include "omni_proc/omni_util.hpp"

omni_mono_proc::omni_mono_proc(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

bool omni_mono_proc::init()
{
    // Pub-Sub
    image_transport::ImageTransport it(this->nh);
    imgSub = it.subscribe("img_src", 1, &omni_mono_proc::imgCallback, this); //Sub
    imgPub = it.advertise("img_rect", 1);

    //start reconfig
    mp_cb = boost::bind(&omni_mono_proc::dr_callback, this, _1, _2);
    mp_server.setCallback(mp_cb);

    //Read & parse
    omni_util omutil;

    if (!omutil.readCalib("/home/xavier/cam_ws/src/omni_proc/param/out_camera_params.xml")) {
        ROS_WARN("FileStorage is empty!");
        return false;
    }

    //Get the matrices
    this->kMat = omutil.parseCalib("camera_matrix", "data", 3, 3);
    this->dMat = omutil.parseCalib("distortion_coefficients", "data", 4, 1);
    this->xiMat = omutil.parseCalib("xi", "-1", 1, 1);

    omutil.fsdone();

    return true;
}

// === CALLBACK & PUBLISHER ===
void omni_mono_proc::dr_callback(const omni_proc::mono_paramConfig& config, const uint32_t& level)
{
    zoomOut = (float)config.ZOOM_OUT_LEVEL;
    aspectRatio = (float)config.OUT_ASPECT;

    switch (config.OUT_MODE) {
    case 1:
        flags_out = cv::omnidir::RECTIFY_CYLINDRICAL;
        break;
    case 2:
        flags_out = cv::omnidir::RECTIFY_STEREOGRAPHIC;
        break;
    case 3:
        flags_out = cv::omnidir::RECTIFY_LONGLATI;
        break;
    case 0:
    default:
        flags_out = cv::omnidir::RECTIFY_PERSPECTIVE;
    }
}

void omni_mono_proc::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };

        //ROS_WARN("\nRectifying IMG...");

        this->new_size.width = imagePtrRaw->image.cols;
        this->new_size.height = imagePtrRaw->image.rows;

        const int centerX{ this->new_size.width / 2 };
        const int centerY{ this->new_size.height / 2 };

        Knew = cv::Matx33f(this->new_size.width / (aspectRatio * zoomOut), 0, centerX,
            0, this->new_size.height / zoomOut, centerY,
            0, 0, 1);

        /*
ROS_WARN_STREAM(imagePtrRaw->image);
ROS_WARN_STREAM(this->kMat);
ROS_WARN_STREAM(this->dMat);
ROS_WARN_STREAM(this->xiMat);
ROS_WARN_STREAM( flags_out);
ROS_WARN_STREAM(Knew);
ROS_WARN_STREAM(this->new_size);
*/

        cv::omnidir::undistortImage(imagePtrRaw->image, imagePtrRaw->image, this->kMat, this->dMat, this->xiMat, flags_out, Knew, this->new_size);

        imgPub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr16", imagePtrRaw->image).toImageMsg());
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr16'.", imgp->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_mono_proc");
    ros::NodeHandle nh;

    omni_mono_proc omp(nh);
    if (!omp.init()) {
        ROS_WARN("Exiting...");
        ros::shutdown();
        return -1;
    }
    ros::spin();

    return 0;
}

