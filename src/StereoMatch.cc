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
        param = Elas::parameters(Elas::setting::ROBOTICS);
        param.disp_min = disp_min;
        param.disp_max = disp_max;
        // param.support_threshold = 0.7;      // 比率测试：最低match VS 次低match
        // param.support_texture = 5;          // 支持点的最小纹理
        // param.candidate_stepsize = 5;       // 用于支持点的sobel特征匹配的邻域半径
        // param.incon_window_size = 5;        // 不连续性窗口的尺寸
        // param.incon_threshold = 5;          // 不连续性窗口内的视差范围阈值
        // param.incon_min_support = 5;        // 不连续性窗口内的最低支持点数量
        // param.add_corners = true;           // 是否添加角点
        // param.grid_size = 20;               // 网格尺寸
        // param.beta = 0.02;                  // 图像相似性度量的参数
        // param.gamma = 3;                    // 先验概率常数
        // param.sigma = 1;                    // 先验概率的标准差
        // param.sradius = 5;                  // 标准差半径
        // param.match_texture = 1;            // 最低纹理
        // param.lr_threshold = 1;             // 左右一致性检验阈值
        // param.speckle_sim_threshold = 1;    // 连通域判断阈值
        // param.speckle_size = 200;           // 连通域噪声尺寸判断阈值
        // param.ipol_gap_width = 3;           // 空洞宽
        // param.filter_median = false;        // 是否中值滤波
        // param.filter_adaptive_mean = true;  // 是否自适应中值滤波
        // param.postprocess_only_left = true; // 是否只对左视差图后处理，设置为True可以节省时间
        // param.subsampling = false;          // 每个两个像素进行视差计算，设置为True可以节省时间，但是传入的D1和D2的分辨率必须为(w/2) x (h/2)
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
