/**
 * Usage ./rgbd_slam root_dir
 * example : ./rgbd_slam ./data/rgbd
*/

#include <iostream>
#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <dirent.h>
#include <System.h>
#include <chrono>
#include <unistd.h>
#include <algorithm>
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
std::string parampath = "/home/ros/lib/ORB_Dense/MyExample/rgbd_orb.yaml";

int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "the number of parameters less than 2 " << std::endl;
        return 0;
    }
    auto rgb_sequence = listdir(std::string(argv[1]) + "/rgb/");
    auto depth_sequence = listdir(std::string(argv[1]) + "/depth/");
    ORB_SLAM3::System SLAM(vocpath, parampath, ORB_SLAM3::System::RGBD, true);
    cv::Mat frame;
    auto start = std::chrono::steady_clock::now();
    // cv::namedWindow("ORB-SLAM3: Current Frame",cv::WINDOW_NORMAL);
    for (int i = 0; i < rgb_sequence.size(); i++)
    {
        cv::Mat rgb = cv::imread(std::string(argv[1]) + "/rgb/" + rgb_sequence[i], cv::IMREAD_UNCHANGED);
        cv::Mat depth = cv::imread(std::string(argv[1]) + "/depth/" + depth_sequence[i], cv::IMREAD_UNCHANGED);
        auto now = std::chrono::steady_clock::now();
        auto time = now - start;
        SLAM.TrackRGBD(rgb, depth, static_cast<double>(time.count()) / 100.0);
    }
    // system("pause");
    sleep(1);
    SLAM.Shutdown();

    return 0;
}