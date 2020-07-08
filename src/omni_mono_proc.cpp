#include "omni_proc/omni_mono_proc.hpp"

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

    if (!readCalib("/home/xavier/cam_ws/src/omni_proc/param/out_camera_params.xml"))
        return false;

    parseCalib(this->fs);
    return true;
}

bool omni_mono_proc::readCalib(const std::string& cfile)
{
    cv::FileStorage fs(cfile, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        ROS_WARN("Error reading calibration file...");
        return false;
    }

    cv::FileNode n = fs.getFirstTopLevelNode();
    if (n.name() != "calibration_time") {
        ROS_WARN("Invalid calibration file!");
        return false;
    }
    this->fs = fs;
    return true;
}

void omni_mono_proc::parseCalib(const cv::FileStorage& fs)
{
    //Get Calibrated Camera matrix (K)
    cv::FileNode mData = fs["camera_matrix"]["data"];
    this->kMat = node2array(mData, 3, 3);

    //Get distortion coeff (D): Distortion parameters (k1,k2,p1,p2)
    cv::FileNode dcf = fs["distortion_coefficients"]["data"];
    this->dMat = node2array(dcf, 4, 1);

    //Get Xi
    cv::FileNode node_xi = fs["xi"];
    this->xiMat = node2array(node_xi, 1, 1);
}

// === CALLBACK & PUBLISHER ===
void omni_mono_proc::imgCallback(const sensor_msgs::ImageConstPtr& imgp)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR16) };

        //ROS_WARN("\nRectifying IMG...");

        constexpr float zoomOut{ 5 }; //Best around 2-6, negative would flip image
        constexpr int flags_out = cv::omnidir::RECTIFY_PERSPECTIVE;
        constexpr float aspectRatio{ 1.7 };

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

// === CV ===
cv::Mat omni_mono_proc::node2array(const cv::FileNode& param, const int& aWidth, const int& aHeight)
{
    int x{ 0 }, y{ 0 };
    double tMat_array[aHeight][aWidth] = {};

    cv::FileNodeIterator it = param.begin(), it_end = param.end();
    for (; it != it_end; ++it) {
        tMat_array[y][x - (3 * y)] = (double)*it; //x have to be offset
        if (((x + 1) % aHeight == 0) && aHeight > 1)
            ++y;
        ++x;
    }
    cv::Mat tMat(aWidth, aHeight, CV_64F);
    std::memcpy(tMat.data, tMat_array, aHeight * aWidth * sizeof(double));

    return tMat;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "omni_mono_proc");
    ros::NodeHandle nh;

    omni_mono_proc omp(nh);
    if (!omp.init())
        return -1;

    ros::spin();

    return 0;
}

