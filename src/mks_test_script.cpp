#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <cstdint>

#include "mks_test.hpp"

constexpr char CAN_INTERFACE[] = "can0";
constexpr char DEFAULT_MOTOR_ID[] = "1";

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;
constexpr uint32_t TOTAL_LOG_SIZE = 100 * 1024 * 1024; // 100 MiB

int main(int argc, const char* argv[]) {
    // Setup logging
    boost::log::add_file_log(
            boost::log::keywords::file_name = "mks_test_script_%Y-%m-%d-%T_%N.log",
            boost::log::keywords::rotation_size = TOTAL_LOG_SIZE,
            boost::log::keywords::format = "[%TimeStamp%]: %Message%",
            boost::log::keywords::auto_flush = true
    );
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);
    boost::log::add_common_attributes();
    BOOST_LOG_TRIVIAL(debug) << "Logging started";

    std::string interface;
    std::vector<uint16_t> motor_ids;
    try {
        boost::program_options::options_description options;
        options.add_options()
            ("interface", boost::program_options::value<std::string>()->default_value(CAN_INTERFACE), "SocketCAN network interface")
            ("motors", boost::program_options::value<std::vector<uint16_t>>()->multitoken()->composing()->required(), "List of CAN IDs for motor controllers to test")
            ("help", "Show help");

        boost::program_options::variables_map vm;
        store(parse_command_line(argc, argv, options), vm);

        // Print help output before notify so that you don't need to specify motors
        if (vm.count("help")) {
            std::cout << options << std::endl;
            return 0;
        }

        notify(vm);

        interface = vm["interface"].as<std::string>();
        motor_ids = vm["motors"].as<std::vector<uint16_t>>();
    }
    catch (const boost::program_options::error& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    MksTest test(interface, std::move(motor_ids), 16);

    // Run update loop forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    for (;;) {
        test.update();
    }

    return 0;
}