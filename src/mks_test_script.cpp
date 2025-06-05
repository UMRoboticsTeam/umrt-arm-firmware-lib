#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <string>

#include "mks_test.hpp"

const std::string CAN_INTERFACE = "can0";

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;
constexpr uint32_t TOTAL_LOG_SIZE = 100 * 1024 * 1024; // 100 MiB

int main() {
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

    MksTest c(CAN_INTERFACE, {1}, 16);

    // Run update loop forever
    // TODO: Look into a better way of doing the polling loop which isn't so intensive
    for (;;) {
        c.update();
    }

    return 0;
}