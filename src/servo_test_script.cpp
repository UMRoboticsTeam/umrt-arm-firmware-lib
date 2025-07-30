//
// Created by Noah on 2025-07-30.
//

#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <cstdint>
#include <chrono>
#include <thread>

#include "servo_controller.hpp"

constexpr char CAN_INTERFACE[] = "can0";

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;
constexpr uint32_t TOTAL_LOG_SIZE = 100 * 1024 * 1024; // 100 MiB

int main(int argc, const char* argv[]) {
    // Setup logging
    boost::log::add_file_log(
            boost::log::keywords::file_name = "servo_controller_test_script_%Y-%m-%d-%T_%N.log",
            boost::log::keywords::rotation_size = TOTAL_LOG_SIZE,
            boost::log::keywords::format = "[%TimeStamp%]: %Message%",
            boost::log::keywords::auto_flush = true
    );
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);
    boost::log::add_common_attributes();
    BOOST_LOG_TRIVIAL(debug) << "Logging started";

    std::string interface;
    uint16_t servo_id;
    try {
        boost::program_options::options_description options;
        options.add_options()
            ("interface,i", boost::program_options::value<std::string>()->default_value(CAN_INTERFACE), "SocketCAN network interface")
            ("servo,s", boost::program_options::value<uint16_t>()->required(), "CAN ID for servo controller to test")
            ("help,h", "Show help");

        boost::program_options::variables_map vm;
        store(parse_command_line(argc, argv, options), vm);

        // Print help output before notify so that you don't need to specify servo
        if (vm.count("help")) {
            std::cout << options << std::endl;
            return 0;
        }

        notify(vm);

        interface = vm["interface"].as<std::string>();
        servo_id = vm["servo"].as<uint16_t>();
    }
    catch (const boost::program_options::error& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    ServoController controller(interface, servo_id);

    // Send min
    controller.send(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Send max
    controller.send(255);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Send middle
    controller.send(127);

    return 0;
}