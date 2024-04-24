/**
 * Usage ./rgbd_slam root_dir
 * example : ./stereoslam_disp ./data/stereo/left ./data/stereo/right ./data/stereo/disp
*/

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <System.h>
#include <chrono>
#include <algorithm>
#include <unistd.h>

class StringSort
{
public:
    bool operator()(const std::string &a, const std::string &b)
    {
        return std::stoi(a.substr(0, a.size() - 4)) < std::stoi(b.substr(0, b.size() - 4));
    }
};

std::vector<std::string> listdir(std::string path)
{
    std::vector<std::string> files;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir(path.c_str())) != nullptr)
    { // 打开当前文件夹
        while ((ent = readdir(dir)) != nullptr)
        { // 读取文件夹中的每个文件
            if (ent->d_type == DT_REG)
            { // 仅处理普通文件
                files.emplace_back(ent->d_name);
            }
        }
        closedir(dir); // 关闭文件夹
    }
    else
    {
        return {};
    }

    std::sort(files.begin(), files.end(), StringSort());
    return files;
}
std::string vocpath = "/home/ros/lib/ORB_Dense/Vocabulary/ORBvoc.txt";
std::string parampath = "/home/ros/lib/ORB_Dense/MyExample/stereoslam_disp.yaml";
int main(int argc, char *argv[])
{
    if (argc < 4)
    {
        std::cout << "the number of parameters less than 2 " << std::endl;
        return 0;
    }
    // 图像路径
    std::string left_path(argv[1]);
    std::string right_path(argv[2]);
    std::string disp_path(argv[3]);
    // 图像名称
    auto l_ps = listdir(left_path);
    auto r_ps = listdir(right_path);
    auto d_ps = listdir(disp_path);

    // slam系统
    ORB_SLAM3::System SLAM(vocpath, parampath, ORB_SLAM3::System::STEREO, true);
    auto start = std::chrono::steady_clock::now();
    std::cout << "image size : " << l_ps.size() << std::endl;
    for (int i = 0; i < l_ps.size(); i++)
    {

        cv::Mat left_img = cv::imread(left_path + l_ps[i], cv::IMREAD_UNCHANGED);
        cv::Mat right_img = cv::imread(right_path + r_ps[i], cv::IMREAD_UNCHANGED);
        cv::Mat disp_img = cv::imread(disp_path + d_ps[i], cv::IMREAD_UNCHANGED);
        if (left_img.empty() || right_img.empty())
        {
            std::cout << "image empty , exit !" << std::endl;
            break;
        }
        // cv::imshow("left",left_img);
        // cv::imshow("right",right_img);
        auto now = std::chrono::steady_clock::now();
        auto time = now - start;
        SLAM.TrackStereo(left_img, right_img, disp_img, static_cast<double>(time.count()) / 1000.0);
        if (cv::waitKey(10) == 27)
            break;
    }
    sleep(1);
    // 关机
    SLAM.Shutdown();

    return 0;
}
