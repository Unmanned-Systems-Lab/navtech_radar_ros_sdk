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
#include "Offline_client.h"
#include "Log.h"
#include "Colossus_TCP_events.h"
#include "string_helpers.h"
#include "configurationdata.pb.h"


namespace Navtech::Networking::Offline {

    using namespace Colossus_protocol;

    using Utility::stdout_log;
    using Utility::endl;

    Client::Client(const std::string& filepath) :
        client { filepath, Colossus_protocol::TCP::Client_event::dispatcher },
        msg_dispatcher { *this, Colossus_protocol::TCP::Client_event::dispatcher },
        event_dispatcher { associate_with<Client::Event_dispatcher>(Colossus_protocol::TCP::Client_event::dispatcher)}
    {
    }


    void Client::start()
    {
        event_handler.when_notified_invoke(
            [this](ID_Ty connection_id)
            {
                msg_dispatcher.stop();
            }
        );
        event_dispatcher->template attach_to<Event_traits::Client_disconnected>(event_handler);

        client.start();
        msg_dispatcher.start();
    }


    void Client::stop()
    {
        msg_dispatcher.stop();
        msg_dispatcher.join();

        client.stop();
        client.join();

        event_dispatcher->template detach_from<Event_traits::Client_disconnected>(event_handler);
    }


    void Client::join()
    {
        msg_dispatcher.join();
        client.join();
    }

    
    void Client::set_handler(TCP::Type type, const Handler& handler)
    {
        msg_dispatcher.attach_to(type, handler);
    }


    void Client::remove_handler(TCP::Type type)
    {
        msg_dispatcher.detach_from(type);
    }
} // namespace Navtech::Utility::Offline