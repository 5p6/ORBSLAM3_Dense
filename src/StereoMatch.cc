#include "StereoMatch.h"

namespace ORB_SLAM3
{

    std::shared_ptr<Stereo_Algorithm> Stereo_Algorithm::create(double disp_min, double disp_max, AlgorithmType type)
    {
        switch (type)
        {
        case Stereo_Algorithm::AlgorithmType::ELAS:
            std::cout << "create elas algorithm" << std::endl;
            return std::make_shared<Elas_Algorithm>(disp_min, disp_max);
        case Stereo_Algorithm::AlgorithmType::SGBM:
            std::cout << "create sgbm algorithm" << std::endl;
            return std::make_shared<SGBM_Algorithm>(disp_min, disp_max);
        default:
            return nullptr;
        }
    }
    // ----------------------------------------------------------------
    // elas algorithm setting
    Elas_Algorithm::Elas_Algorithm(double disp_min, double disp_max)
    {
        param = Elas::parameters(Elas::setting::MIDDLEBURY);
        param.disp_min = disp_min;
        param.disp_max = disp_max;       
        // model
        model = std::make_unique<Elas>(param);
    }

    // disparity inference
    cv::Mat Elas_Algorithm::inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified)
    {
        cv::Mat left = left_rectified.clone();
        cv::Mat right = right_rectified.clone();
        // 参数调整
        int height = left.rows;
        int width = left.cols;
        int dim[3] = {width, height, width};
        if (left.channels() == 3)
            cv::cvtColor(left, left, cv::COLOR_BGR2GRAY);
        if (right.channels() == 3)
            cv::cvtColor(right, right, cv::COLOR_BGR2GRAY);
        cv::Mat left_disp = cv::Mat::zeros(left.size(), CV_32FC1);
        cv::Mat right_disp = cv::Mat::zeros(right.size(), CV_32FC1);
        // 计算
        model->process(left.data, right.data, left_disp.ptr<float>(0), right_disp.ptr<float>(0), dim);

        return left_disp;
    }

    // ----------------------------------------------------------------
    // sgbm

    SGBM_Algorithm::SGBM_Algorithm(double disp_min, double disp_max)
    {
        int block_size = 8;
        int P1 = 8 * 3 * block_size * block_size;
        int P2 = 32 * 3 * block_size * block_size;
        // create the stereo
        model = cv::StereoSGBM::create(
            disp_min, disp_max, block_size, P1, P2, -1, 1, 10, 100, 1, cv::StereoSGBM::MODE_HH);

#ifdef WITH_FILTER
        wlsfilter = cv::ximgproc::createDisparityWLSFilter(model);
        model_right = cv::ximgproc::createRightMatcher(model);
        wlsfilter->setLambda(8000.0);
        wlsfilter->setSigmaColor(1.20);
#endif
    }
    cv::Mat SGBM_Algorithm::inference(const cv::Mat &left_rectified, const cv::Mat &right_rectified)
    {
        cv::Mat disp;
        model->compute(left_rectified, right_rectified, disp);
#ifdef WITH_FILTER
        cv::Mat right_disp;
        model_right->compute(right_rectified,left_rectified,right_disp);
        wlsfilter->filter(disp,left_rectified,disp,right_disp);
#endif
        return disp / 16;
    }
}
