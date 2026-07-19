#include "KLTTracker.h"
#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>

namespace ORB_SLAM3
{

KLTTracker::KLTTracker(int maxLevel, float epsilon, bool checkBrightness)
    : mMaxLevel(maxLevel),
      mEpsilon(epsilon),
      mbInitialized(false)
{
}

std::vector<cv::KeyPoint> KLTTracker::Track(
    const cv::Mat &prevImg,
    const cv::Mat &currImg,
    const std::vector<cv::KeyPoint> &prevKeys,
    std::vector<int> &vMatchedIndices)
{
    std::vector<cv::KeyPoint> trackedKeys;
    vMatchedIndices.clear();

    if(prevImg.empty() || prevKeys.empty())
        return trackedKeys;

    // Convert keypoints to points
    std::vector<cv::Point2f> prevPts, currPts;
    for(const auto &kp : prevKeys)
        prevPts.push_back(kp.pt);

    // Run OpenCV's optimized KLT — processes all points in one batch
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(
        prevImg, currImg,
        prevPts, currPts,
        status, err,
        cv::Size(21, 21),  // window size
        mMaxLevel,         // pyramid levels
        cv::TermCriteria(
            cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
            30, mEpsilon
        )
    );

    // Keep only successfully tracked points
    for(size_t i = 0; i < status.size(); i++)
    {
        if(status[i])
        {
            cv::KeyPoint kp = prevKeys[i];
            kp.pt = currPts[i];
            trackedKeys.push_back(kp);
            vMatchedIndices.push_back(i);
        }
    }

    std::cout << "[KLT] Tracked " << trackedKeys.size()
              << " / " << prevKeys.size() << " points" << std::endl;

    mbInitialized = true;
    return trackedKeys;
}

} // namespace ORB_SLAM3