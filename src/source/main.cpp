#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <RobotController.hpp>

namespace {

bool hasParamsFile(int argc, char ** argv)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--params-file" || arg.rfind("--params-file=", 0) == 0) {
            return true;
        }
    }
    return false;
}

}  // namespace

int main(int argc, char * argv[])
{
    std::cout << "Controller started." << std::endl;

    std::vector<std::string> injected_args;
    std::vector<char *> argv_ptrs;
    argv_ptrs.reserve(static_cast<size_t>(argc) + 4);
    for (int i = 0; i < argc; ++i) {
        argv_ptrs.push_back(argv[i]);
    }

    if (!hasParamsFile(argc, argv)) {
        try {
            const auto config = ament_index_cpp::get_package_share_directory("wave_rover_controller")
                + "/config/wave_rover_controller.yaml";
            injected_args = {"--ros-args", "--params-file", config};
            for (auto & arg : injected_args) {
                argv_ptrs.push_back(arg.data());
            }
            std::cout << "Loading default config: " << config << std::endl;
        } catch (const std::exception & e) {
            std::cerr << "Warning: could not locate package config, using built-in defaults: "
                      << e.what() << std::endl;
        }
    }

    const int effective_argc = static_cast<int>(argv_ptrs.size());
    rclcpp::init(effective_argc, argv_ptrs.data());

    RobotController robot;
    while (rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}
