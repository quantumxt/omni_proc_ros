#ifndef CV_HEADER
#include "opencv2/core.hpp"
#endif

class omni_util {
public:
    omni_util();

    bool readCalib(const std::string& cfile);
    cv::Mat parseCalib(const std::string& id, const std::string& type, const int& mWidth, const int& mHeight);
    void fsdone();

private:
    cv::FileStorage fs;
    cv::Mat node2array(const cv::FileNode& param, const int& aWidth, const int& aHeight);
};

