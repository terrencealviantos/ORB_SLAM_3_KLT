#ifndef KLTTRACKER_H
#define KLTTRACKER_H

#include <opencv2/core/core.hpp>
#include <opencv2/video/tracking.hpp>
#include <vector>

namespace ORB_SLAM3
{

// Sparse pyramidal Lucas-Kanade tracker used to track the keypoints of the last frame
// into the current one, so ORB features only need to be extracted on keyframe candidates.
// The pyramid of the current image is kept as the reference for the next frame, so every
// image pyramid is built only once.
class KLTTracker
{
public:
    KLTTracker(int winSize = 21, int maxLevel = 3, int maxIterations = 30, float epsilon = 0.01f, float maxFBError = 1.0f);

    // Track prevPts from the reference image into currImg. status[i] is 1 if the point was tracked,
    // stays inside the image and passes the forward-backward check (when enabled).
    void Track(const cv::Mat &currImg, const std::vector<cv::Point2f> &prevPts,
               std::vector<cv::Point2f> &currPts, std::vector<uchar> &status);

    // Find in imRight the correspondences of points of the image given to the last Track() call
    // (the left image of a stereo pair). rightPts holds the initial guesses on input.
    void TrackStereo(const cv::Mat &imRight, const std::vector<cv::Point2f> &leftPts,
                     std::vector<cv::Point2f> &rightPts, std::vector<uchar> &status);

    // Make im the reference image for the next Track() call. If im was just tracked with Track(),
    // its pyramid is reused.
    void SetReference(const cv::Mat &im);

    bool HasReference() const { return !mvRefPyr.empty(); }

private:
    void BuildPyramid(const cv::Mat &im, std::vector<cv::Mat> &pyr) const;

    cv::Size mWinSize;
    int mMaxLevel;
    cv::TermCriteria mTermCrit;
    float mMaxFBError;

    // Pyramids of the reference (previous) image and of the image given to the last Track() call
    std::vector<cv::Mat> mvRefPyr, mvCurrPyr;
    bool mbCurrPyrValid;
};

} // namespace ORB_SLAM3

#endif // KLTTRACKER_H
