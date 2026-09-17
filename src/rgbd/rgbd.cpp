#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rgbd-slam-node.hpp"

#include "System.h"

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cerr << "\nUsage: ros2 run orbslam rgbd path_to_vocabulary path_to_settings" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    // malloc error using new.. try shared ptr
    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    bool visualization = true;
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::RGBD, visualization);

    auto node = std::make_shared<RgbdSlamNode>(&SLAM);
    std::cout << "============================ " << std::endl;

    rclcpp::spin(node);
    node.reset(); // runs ~RgbdSlamNode() now: SLAM->Shutdown() + trajectory save
    rclcpp::shutdown();

    // ORB_SLAM3's Viewer thread (Pangolin GL + OpenCV/GTK window) is never
    // joined and keeps running after Shutdown(). Returning normally from
    // main() runs global/static destructors (e.g. the DDS layer's) while
    // that orphaned thread is still mid GL/GTK call, which segfaults inside
    // the NVIDIA EGL driver or libgdk on this machine. Everything we need
    // (trajectory, Atlas) is already saved above, so skip that teardown
    // entirely instead of trying to make the orphaned thread exit safely.
    _exit(0);
}
