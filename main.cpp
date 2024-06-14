#include <boost/log/trivial.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <iostream>
#include <string>

#include "StepperController.h"

const std::string DEVICE = "/dev/ttyUSB1";
const int BAUD_RATE = 57600;

constexpr boost::log::trivial::severity_level LOG_LEVEL = boost::log::trivial::debug;
constexpr uint32_t TOTAL_LOG_SIZE = 100 * 1024 * 1024; // 100 MiB

void onSetup();


int main() {
    // Setup logging
    boost::log::add_file_log(
            boost::log::keywords::file_name = "[%TimeStamp%]_%N.log",
            boost::log::keywords::rotation_size = TOTAL_LOG_SIZE,
            boost::log::keywords::format = "[%TimeStamp%]: %Message%",
            boost::log::keywords::auto_flush = true
    );
    boost::log::core::get()->set_filter(boost::log::trivial::severity >= LOG_LEVEL);
    boost::log::add_common_attributes();
    BOOST_LOG_TRIVIAL(debug) << "Logging started";

    StepperController s;

    s.ESetup.connect(&onSetup);

    s.connect(DEVICE, 57600);

    // Run update loop forever
    for (;;) {
        s.update();
    }

    return 0;
}

void onSetup(){
    std::cout << "Arduino setup!" << std::endl;

}