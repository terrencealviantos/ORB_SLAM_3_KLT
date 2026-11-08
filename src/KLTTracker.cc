#include "KLTTracker.h"

namespace ORB_SLAM3
{

KLTTracker::KLTTracker(int winSize, int maxLevel, int maxIterations, float epsilon, float maxFBError)
    : mWinSize(winSize, winSize),
      mMaxLevel(maxLevel),
      mTermCrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, maxIterations, epsilon),
      mMaxFBError(maxFBError),
      mbCurrPyrValid(false)
{
}

void KLTTracker::BuildPyramid(const cv::Mat &im, std::vector<cv::Mat> &pyr) const
{
    cv::buildOpticalFlowPyramid(im, pyr, mWinSize, mMaxLevel);
}

void KLTTracker::Track(const cv::Mat &currImg, const std::vector<cv::Point2f> &prevPts,
                       std::vector<cv::Point2f> &currPts, std::vector<uchar> &status)
{
    currPts.clear();
    status.assign(prevPts.size(), 0);

    BuildPyramid(currImg, mvCurrPyr);
    mbCurrPyrValid = true;

    if(mvRefPyr.empty() || prevPts.empty())
        return;

    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(mvRefPyr, mvCurrPyr, prevPts, currPts, status, err,
                             mWinSize, mMaxLevel, mTermCrit);

    // Forward-backward check: track the points back and reject those that do not return to where they started
    std::vector<cv::Point2f> backPts;
    std::vector<uchar> backStatus;
    if(mMaxFBError > 0)
    {
        backPts = prevPts;
        cv::calcOpticalFlowPyrLK(mvCurrPyr, mvRefPyr, currPts, backPts, backStatus, err,
                                 mWinSize, mMaxLevel, mTermCrit, cv::OPTFLOW_USE_INITIAL_FLOW);
    }

    const float maxFBError2 = mMaxFBError*mMaxFBError;
    const float maxX = currImg.cols-1, maxY = currImg.rows-1;
    for(size_t i=0; i<prevPts.size(); i++)
    {
        if(!status[i])
            continue;

        const cv::Point2f &pt = currPts[i];
        if(pt.x<0 || pt.y<0 || pt.x>=maxX || pt.y>=maxY)
        {
            status[i] = 0;
            continue;
        }

        if(mMaxFBError > 0)
        {
            const cv::Point2f d = backPts[i]-prevPts[i];
            if(!backStatus[i] || d.dot(d)>maxFBError2)
                status[i] = 0;
        }
    }
}

void KLTTracker::TrackStereo(const cv::Mat &imRight, const std::vector<cv::Point2f> &leftPts,
                             std::vector<cv::Point2f> &rightPts, std::vector<uchar> &status)
{
    status.assign(leftPts.size(), 0);
    if(!mbCurrPyrValid || leftPts.empty())
        return;

    std::vector<cv::Mat> vRightPyr;
    BuildPyramid(imRight, vRightPyr);

    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(mvCurrPyr, vRightPyr, leftPts, rightPts, status, err,
                             mWinSize, mMaxLevel, mTermCrit, cv::OPTFLOW_USE_INITIAL_FLOW);

    const float maxX = imRight.cols-1, maxY = imRight.rows-1;
    for(size_t i=0; i<leftPts.size(); i++)
    {
        const cv::Point2f &pt = rightPts[i];
        if(status[i] && (pt.x<0 || pt.y<0 || pt.x>=maxX || pt.y>=maxY))
            status[i] = 0;
    }
}

void KLTTracker::SetReference(const cv::Mat &im)
{
    if(mbCurrPyrValid)
        std::swap(mvRefPyr, mvCurrPyr);
    else
        BuildPyramid(im, mvRefPyr);

    mbCurrPyrValid = false;
}

} // namespace ORB_SLAM3
