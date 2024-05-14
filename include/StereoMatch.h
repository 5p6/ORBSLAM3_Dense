#ifndef STEREOMATCH_H
#define STEREOMATCH_H

#include <iostream>
#include <opencv2/opencv.hpp>

#ifdef WITH_FILTER
#include <opencv2/ximgproc/disparity_filter.hpp>
#endif

#include "Thirdparty/elas/elas/elas.h"

namespace ORB_SLAM3
{
    // base class for sterep matching
    class Stereo_Algorithm
    {
    public:
    enum AlgorithmType{
        ELAS = 0,
        SGBM = 1
    };
    public:
        //
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified) = 0;

        static std::shared_ptr<Stereo_Algorithm> create(double disp_min, double disp_max,AlgorithmType type);
    
    };


    // disparity calculation
    class Elas_Algorithm : public Stereo_Algorithm
    {
    public:
        Elas_Algorithm(double disp_min, double disp_max);
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified);

    protected:
        Elas::parameters param;
        std::unique_ptr<Elas> model;
    };

    class SGBM_Algorithm : public Stereo_Algorithm
    {
    public:
        SGBM_Algorithm(double disp_min, double disp_max);
        virtual cv::Mat inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified);
    public:
        cv::Ptr<cv::StereoSGBM> model;
#ifdef WITH_FILTER
    // filter
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wlsfilter;
    cv::Ptr<cv::StereoMatcher> model_right;
#endif
        
    };

}

#endif