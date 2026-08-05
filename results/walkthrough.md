# Walkthrough: KLT Hybrid Tracking Integration in ORB-SLAM3

We have successfully integrated a KLT (Kanade-Lucas-Tomasi) optical flow tracking algorithm into ORB-SLAM3, validated its performance on the TUM dataset, instrumented it with scoped timers, and generated single-frame and multi-frame execution Gantt charts.

---

## 1. Summary of Changes

### KLT Tracker Module
- **[KLTTracker.h](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/include/KLTTracker.h) & [KLTTracker.cc](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/src/KLTTracker.cc)**: Modified `Track` signature to return `vMatchedIndices` by reference, maintaining a strict 1-to-1 index map between tracked features in the current frame and their keypoints / descriptors / `MapPoints` from the previous frame.

### Frame Management & Selective Feature Extraction
- **[Frame.h](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/include/Frame.h) & [Frame.cc](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/src/Frame.cc)**:
  - Extended the monocular `Frame` constructor to accept an optional `bool bExtractORB = true` parameter.
  - When `bExtractORB = false`, ORB feature detection and description computation are bypassed, avoiding CPU overhead on non-keyframe frames.
  - Implemented `InitFromKLT(...)` to initialize tracked keypoint positions, copy descriptors and `MapPoint` references from the previous frame, undistort keypoints, and assign them to grid bins.
  - Added `mbIsKLTFrame` flag.

### Tracking Engine & Pose Optimization
- **[Tracking.h](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/include/Tracking.h) & [Tracking.cc](file:///home/terrence/cancelled/researchProjectKLT/ORB_SLAM3/src/Tracking.cc)**:
  - Updated `GrabImageMonocular` to run KLT inter-frame tracking for non-keyframes, creating KLT frames without ORB extraction.
  - Implemented `TrackWithKLT()` which estimates initial camera pose via constant velocity model and optimizes camera pose using g2o Bundle Adjustment on the KLT tracked landmarks.
  - Updated `NeedNewKeyFrame()` to return `false` on KLT frames.
  - Instrumented code blocks with a custom RAII `ScopedTimer` to capture execution times of core modules.

---

## 2. Multi-Frame Gantt Chart Timeline (10 Consecutive Frames)

Below is the multi-frame Gantt chart showing the exact sub-task execution timeline across 10 consecutive video frames ($F_0$ to $F_9$):

![Multi-Frame Gantt Chart](/home/terrence/.gemini/antigravity/brain/0dd363fd-814a-45c5-bce5-a346ec555f5f/multi_frame_gantt.png)

---

## 3. GitHub Repository Upload

The updated codebase has been committed and pushed to your GitHub repository:
- **Repository URL**: [https://github.com/terrencealviantos/ORB_SLAM_3_KLT](https://github.com/terrencealviantos/ORB_SLAM_3_KLT)
- **Branch**: `master`
- **Latest Commits**:
  - `Implement RAII ScopedTimer for Tracking modules`
  - `Update README with Hybrid KLT explanation, benchmark results, and Docker usage instructions`
