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
#ifndef STREAM_CLIENT_H
#define STREAM_CLIENT_H

#ifdef _WIN32
#include <Windows.h>
#include <WinSock2.h>
#endif

#include "Stream_client_traits.h"
#include "Event_traits.h"

#include "Active.h"
#include "Endpoint.h"
#include "Time_utils.h"
#include "Log.h"

#include "Message_dispatchers.h"
#include "Connection_manager.h"


using Navtech::Utility::stdout_log;
using Navtech::Utility::endl;
using Navtech::Utility::Logging_level;

namespace Navtech::Networking {

    // The Stream_client manages the connection, retry and disconnect
    // logic of a server connection.  The connect logic is implemented
    // as a finite state machine.
    //
    template <Protocol protocol, Transport transport, TLS::Type tls = TLS::Type::none>
    class Stream_client : public Utility::Active {
    public:
        // Type aliases.
        // The '_Ty' postfix denotes a template type. Rather than being
        // supplied as template parameters on the class (which would be
        // unwieldy) these parameters are looked up from the Networking::Traits
        // class, using the appropriate combination of protocol and transport
        //
        using Traits            = Navtech::Networking::Stream_client_traits<protocol, transport, tls>;
        using Event_traits      = Navtech::Networking::Event_traits<protocol, transport, tls>;
        using Socket_Ty         = typename Traits::Socket;
        using Socket_ptr_Ty     = typename Traits::Socket_ptr;
        using ID_Ty             = typename Traits::ID;
        using Message_Ty        = typename Traits::Message;
        using Dispatcher_Ty     = typename Event_traits::Dispatcher;
        
        Stream_client(Dispatcher_Ty& event_dispatcher);

        Stream_client(
            const IP_address&   ip_address,
            Port                port,
            Dispatcher_Ty&      event_dispatcher
        );

        Stream_client(
            const IP_address&       ip_address,
            Port                    port,
            std::uint16_t           retries,
            const Time::Duration&   retry_wait_time,
            Dispatcher_Ty&          event_dispatcher
        );

        void send(const Message_Ty& msg);
        void send(Message_Ty&& msg);

    private:
        // External associations
        //
        association_to<Dispatcher_Ty> dispatcher;

        Endpoint        endpoint        { };
        std::uint16_t   retry_allowance { std::numeric_limits<std::uint16_t>::infinity() };
        Time::Duration  wait_period     { 5_sec };

        Socket_ptr_Ty socket { };
        Connection_manager<protocol, transport, tls> connection_mgr;

        std::uint16_t retry_count { 0 };
        bool enabled { false };

        void on_start() override;
        void on_stop()  override;

        // Event handling
        //
        Error::Dispatcher             error_events  { };
        Utility::Event_handler<ID_Ty> error_handler { };
        void on_error(ID_Ty connection_id);

        // Finite State Machine implementation
        //
        enum Event    { open, connect_fail, retry, failed, connect_ok, error, num_events };
        enum State    { disconnected, connecting, waiting, connected, num_states };

        using Activity = void (Stream_client::*)(void);

        struct State_cell {
            State    next_state;
            Activity do_action;
        };

        State current_state { disconnected };

        void post_event(Event e) { async_call(&Stream_client::process_event, this, e); }

        void disconnect();
        void connect();
        void wait();
        void open_connection();
        void initialise_socket();
        void process_event(Event event);

        static constexpr State_cell state_machine[num_events][num_states] {
        /*              Disconnected                             Connecting                                     Waiting                                      Connected */
        /* open    */ { { connecting, &Stream_client::connect }, { },                                            { },                                          { } },
        /* fail    */ { { },                                     { waiting,   &Stream_client::wait },            { },                                          { } },
        /* retry   */ { { },                                     { },                                            { connecting,   &Stream_client::connect },    { } },
        /* failed  */ { { },                                     { },                                            { disconnected, &Stream_client::disconnect }, { } },
        /* conn OK */ { { },                                     { connected, &Stream_client::open_connection }, { },                                          { } },
        /* error   */ { { },                                     { },                                            { },                                          { waiting, &Stream_client::wait } }        
        };
    };


    // -----------------------------------------------------------------------------------------------------------------
    //
    template <Protocol protocol, Transport transport, TLS::Type tls>
    Stream_client<protocol, transport, tls>::Stream_client(Stream_client<protocol, transport, tls>::Dispatcher_Ty& event_dispatcher) :
        Active          { "Radar client" },
        dispatcher      { associate_with(event_dispatcher) },
        connection_mgr  { event_dispatcher }
    {
        // initialise_socket();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Stream_client<protocol, transport, tls>::Stream_client(
        const IP_address&                                   ip_address,
        Port                                                port,
        Stream_client<protocol, transport, tls>::Dispatcher_Ty& event_dispatcher
    ) :
        Active          { "Radar client" },
        dispatcher      { associate_with(event_dispatcher) },
        endpoint        { ip_address, port },
        connection_mgr  { event_dispatcher }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    Stream_client<protocol, transport, tls>::Stream_client(
        const IP_address&                                   ip_address,
        Port                                                port,
        std::uint16_t                                       retries,
        const Duration&                                     retry_wait_time,
        Stream_client<protocol, transport, tls>::Dispatcher_Ty& event_dispatcher
    ) :
        Active          { "Radar client" },
        dispatcher      { associate_with(event_dispatcher) },
        endpoint        { ip_address, port },
        retry_allowance { retries },
        wait_period     { retry_wait_time },
        connection_mgr  { event_dispatcher }
    {
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::send(const Stream_client<protocol, transport, tls>::Message_Ty& msg)
    {
        auto& connections = connection_mgr.all_connections();
        connections.send(msg);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::send(Stream_client<protocol, transport, tls>::Message_Ty&& msg)
    {
        auto& connections = connection_mgr.all_connections();
        connections.send(std::move(msg));
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::on_start()
    {
        error_handler.when_notified_invoke(
            [this](ID_Ty connection_id) 
            { 
                async_call(&Stream_client::on_error, this, connection_id); 
            }
        );
        dispatcher->template attach_to<Event_traits::Client_disconnected>(error_handler);

        connection_mgr.start();

        post_event(open);
    }
    
    
    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::on_stop()
    {
        enabled = false;
      
        dispatcher->template detach_from<Event_traits::Client_disconnected>(error_handler);

        connection_mgr.stop();
        connection_mgr.join();
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::on_error(Stream_client<protocol, transport, tls>::ID_Ty connection_id [[maybe_unused]])
    {
        enabled = false;
        post_event(error);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::disconnect()
    {
        stdout_log << "Radar client disconnected" << endl;

        // TODO - inform other parts of the system?
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::connect()
    {
        initialise_socket();

        try {
            stdout_log << "Connecting to " << "[" << endpoint.to_string() << "]" << endl;

            socket->connect_to(endpoint);
            post_event(connect_ok);
            retry_count = 0;
        }
        catch (std::system_error&) {
            stdout_log << "Radar client - connection to server failed" << endl;
            post_event(connect_fail);
        }
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::wait()
    {
        ++retry_count;

        if (retry_count == retry_allowance) {
            stdout_log << "Radar client - tried to connect " << retry_count << " times. "
                       << "Exceeded number of retries." << endl;
            post_event(connect_fail);
            return;
        }

        if (retry_allowance == std::numeric_limits<std::uint16_t>::infinity()) {
             stdout_log << "Radar client - retry " << retry_count << ". ";
        }
        else {
            stdout_log << "Radar client - retry " << retry_count << " of " << retry_allowance << ". ";
        }
        stdout_log << "Trying again in " << wait_period.to_string() << endl;

        // Split the wait period into chunks to reduce
        // the latency of responding to events
        //
        auto n = wait_period / Traits::min_dwell;

        for (int i { 0 }; i < n; ++i) {
            try_dispatch_async();
            Time::Monotonic::sleep_for(Traits::min_dwell);
        }

        post_event(retry); 
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::open_connection()
    {
        connection_mgr.create_connection(std::move(socket));
        enabled = true;

        stdout_log << "Radar client - connected" << endl;
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::initialise_socket()
    {
        stdout_log << "Initialising socket" << endl;

        socket = Traits::allocate_socket();
        socket->rx_timeout(7_sec);
    }


    template <Protocol protocol, Transport transport, TLS::Type tls>
    void Stream_client<protocol, transport, tls>::process_event(Event event)
    {
        if (!state_machine[event][current_state].do_action) return;

        auto activity = state_machine[event][current_state].do_action;
        current_state = state_machine[event][current_state].next_state;

        (this->*activity)();
    }

} // namespace Navtech::Networking

#endif // STREAM_CLIENT_H
