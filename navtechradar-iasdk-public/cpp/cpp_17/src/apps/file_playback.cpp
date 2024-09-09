// ---------------------------------------------------------------------------------------------------------------------
// Copyright 2024 Navtech Radar Limited
// This file is part of IASDK which is released under The MIT License (MIT).
// See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
// for full license details.
//
// Disclaimer:
// Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
// any warranty of the item whatsoever, whether express, implied, or statutory,
// including, but not limited to, any warranty of merchantability or fitness
// for a particular purpose or any warranty that the contents of the item will
// be error-free.
// In no respect shall Navtech Radar incur any liability for any damages, including,
// but limited to, direct, indirect, special, or consequential damages arising
// out of, resulting from, or any way connected to the use of the item, whether
// or not based upon warranty, contract, tort, or otherwise; whether or not
// injury was sustained by persons or property or otherwise; and whether or not
// loss was sustained from, or arose out of, the results of, the item, or any
// services that may be provided by Navtech Radar.
// ---------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------
// This is a demonstration of the file reader, featuring a playback system.
// ---------------------------------------------------------------------------------------------------------------------
#include "sdk.h"
#include "Option_parser.h"
#include "Endpoint.h"
#include "Time_utils.h"
#include "Log.h"
#include "Signal_handler.h"

#include "Offline_client.h"
#include "Colossus_TCP_messages.h"
#include "health.pb.h"
#include "configurationdata.pb.h"
#include "Protobuf_helpers.h"

using namespace Navtech;
using namespace Navtech::Time;
using namespace Navtech::Time::Monotonic;
using namespace Navtech::Networking::Colossus_protocol;

using Navtech::Utility::Option_parser;
using Navtech::Utility::Option;

using Navtech::Networking::IP_address;
using Navtech::Networking::Port;

using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

using Navtech::Utility::Signal_handler;
using Navtech::Networking::Offline::Client;

// ---------------------------------------------------------------------------------------------------------------------
//
Option_parser options {
    {
        Option { "--file",      "-f",   "playback recording", required, has_argument, "" },
    }
};


// ---------------------------------------------------------------------------------------------------------------------
// Signal handling: If SIGINT or SIGTERM are sent to the 
// program, stop processing.
//
volatile bool running { true };

void stop_running(std::int32_t signal [[maybe_unused]], std::int32_t info [[maybe_unused]])
{
    running = false;
}

// ---------------------------------------------------------------------------------------------------------------------
//
int main(int argc, char* argv[])
{
    using Navtech::Protobuf::from_vector_into;
    using Colossus::Protobuf::ConfigurationData;
    using Colossus::Protobuf::Health;

    SDK::initialise();

    // Set up signal handling for ctrl-c (SIGINT)
    // and kill (SIGTERM)
    // 
    Signal_handler signal_handler { };
    signal_handler.register_handler(SIGINT, stop_running);
    signal_handler.register_handler(SIGTERM, stop_running);

    // Command line option parsing
    //
    options.parse(argc, argv);
    auto filepath = options.global_option("-f").value();

    Client client { filepath };
    
    client.set_handler(TCP::Type::configuration, [](Client&, TCP::Message& msg) { 
        auto cfg = msg.view_as<TCP::Configuration>();
        auto cfg_data = from_vector_into<ConfigurationData>(cfg->to_vector());
        
        stdout_log << "Configuration Message Received" << endl;
        stdout_log << "Serial No [" << cfg_data->radaruniqueid() << "]" << endl;
    });

    client.set_handler(TCP::Type::fft_data, [](Client&, TCP::Message& msg) {
        return;
    });


    client.set_handler(TCP::Type::health, [](Client&, TCP::Message& msg) {
        auto health = msg.view_as<TCP::Health>();
        auto health_data = from_vector_into<Health>(health->to_vector());

        auto to_string = [](Colossus::Protobuf::HealthStatus h) -> std::string
        {
            std::string strings[] { "UNHEALTHY", "WARNING", "HEALTHY", "UNKNOWN" };
            return strings[static_cast<int>(h)];
        };

        stdout_log << "Health Message received." << endl;
        stdout_log << "Status [" << to_string(health_data->dietemperature().status()) << "]" << endl;
    });
    
    client.start();
    
    client.join();

    SDK::shutdown();
    stdout_log << "Done." << endl;
}