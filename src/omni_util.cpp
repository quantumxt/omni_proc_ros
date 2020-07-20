#include "omni_proc/omni_util.hpp"

omni_util::omni_util(){};

void omni_util::fsdone()
{
    this->fs.release();
};

bool omni_util::readCalib(const std::string& cfile)
{
    cv::FileStorage fs(cfile, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        //ROS_WARN("Error reading calibration file...");
        return false;
    }

    cv::FileNode n = fs.getFirstTopLevelNode();
    if (n.name() != "calibration_time") {
        //ROS_WARN("Invalid calibration file!");
        return false;
    }
    this->fs = fs;
    fs.release();
    return true;
}

cv::Mat omni_util::parseCalib(const std::string& id, const std::string& type, const int& mWidth, const int& mHeight)
{
    cv::FileNode mData = this->fs[id];
    if (type != "-1")
        mData = mData[type];
    return node2array(mData, mWidth, mHeight);
}

// === CV ===
cv::Mat omni_util::node2array(const cv::FileNode& param, const int& aWidth, const int& aHeight)
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

