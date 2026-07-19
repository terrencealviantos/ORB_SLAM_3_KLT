#ifndef KLTTRACKER_H
#define KLTTRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace ORB_SLAM3
{

class KLTTracker
{
public:
    KLTTracker(int maxLevel = 3, float epsilon = 0.03f, bool checkBrightness = false);

    // Track keypoints from previous to current frame using OpenCV's LK optical flow
    std::vector<cv::KeyPoint> Track(
        const cv::Mat &prevImg,
        const cv::Mat &currImg,
        const std::vector<cv::KeyPoint> &prevKeys,
        std::vector<int> &vMatchedIndices
    );

    bool IsInitialized() { return mbInitialized; }

private:
    bool mbInitialized;
    int mMaxLevel;
    float mEpsilon;
};

} // namespace ORB_SLAM3

#endif // KLTTRACKER_H