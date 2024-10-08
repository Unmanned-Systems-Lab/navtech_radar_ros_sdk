﻿/*
Copyright 2024 Navtech Radar Limited
This file is part of IASDK which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT
for full license details.

Disclaimer:
Navtech Radar is furnishing this item "as is". Navtech Radar does not provide 
any warranty of the item whatsoever, whether express, implied, or statutory,
including, but not limited to, any warranty of merchantability or fitness
for a particular purpose or any warranty that the contents of the item will
be error-free.
In no respect shall Navtech Radar incur any liability for any damages, including,
but limited to, direct, indirect, special, or consequential damages arising
out of, resulting from, or any way connected to the use of the item, whether
or not based upon warranty, contract, tort, or otherwise; whether or not
injury was sustained by persons or property or otherwise; and whether or not
loss was sustained from, or arose out of, the results of, the item, or any
services that may be provided by Navtech Radar.
*/

using Navtech.IASDK.Events;
using Navtech.IASDK.Networking;
using Navtech.IASDK.Networking.Messages;
using System;
using System.Linq;
using System.Threading;

namespace Navtech.TestClient
{
    /// <summary>
    /// This console application is a very simple example of a radar client application.
    /// It connects to a radar when the application starts and then responds to the following messages:
    ///  - Configuration Message -> Displays some radar config params after initial connection. It also issues a start FFT Data message once the config is received
    ///  - FFT Data -> When a FFT Data message is received we display the radar rotation speed based on the frequency of messages arriving
    /// Data is only processed for 6 seconds before the application closes
    /// </summary>
    internal class Program
    {
        private static readonly RadarTcpClient RadarTcpClient = new RadarTcpClient();
        private static ushort _lastAzimuth;
        private static ushort _PacketCount;

        private static DateTime _lastRotationReset;

        private static void Main(string[] args)
        {
            Console.WriteLine("Test Client Starting");
            //Configure the event handlers for data messages and connection state change
            RadarTcpClient.OnConfigurationData += ConfigurationDataHandler;
            RadarTcpClient.OnFftData += FftDataHandler;
            RadarTcpClient.OnNavigationData += NavigationDataHandler;
            RadarTcpClient.OnConnectionChanged += ConnectionChangedHandler;
            RadarTcpClient.OnNavigationAreaRules += NavigationAreaRuleHandler;
            RadarTcpClient.OnNavigationAlarm += NavigationAlarmHandler;

            //Connect to the radar on default IP address 192.168.0.1
            RadarTcpClient.Connect("10.77.2.211");

            //Sleep for 6 seconds to allow us to process some data
            Thread.Sleep(6000);

            Console.WriteLine("Test Client Stopping");
            //Send a message telling the radar to stop sending FFT Data
            RadarTcpClient.StopFftData();
            //RadarTcpClient.StopNavigationData();

            Thread.Sleep(1000);

            //Disconnect the client from the radar
            RadarTcpClient.Disconnect();

            Console.WriteLine("Test Client Stopped");
        }

        private static void NavigationDataHandler(object sender, NavigationDataEventArgs e)
        {
            Console.WriteLine($"Targets Found [{e.Data.NavigationReturns.Count}] First Target [{e.Data.NavigationReturns.First().Range}, {e.Data.NavigationReturns.First().Power / 10}]");
        }

        //FFT Data handler - handles incoming FFT Data messages
        private static void FftDataHandler(object sender, GenericEventArgs<FftData> fftEventArgs)
        {
            _PacketCount++;
            //Calculate the time difference between FFT Data messages and report the
            //rotation speed
            if (fftEventArgs.Payload.Message.Azimuth < _lastAzimuth)
            {
                var diff = DateTime.UtcNow - _lastRotationReset;
                Console.WriteLine($"FftDataHandler - Rotating @ [{ 1000.0 / diff.TotalMilliseconds:0.00}Hz] Packets [{_PacketCount}]");
                _lastRotationReset = DateTime.UtcNow;
                _PacketCount = 0;
            }

            _lastAzimuth = fftEventArgs.Payload.Message.Azimuth;
        }

        private static void NavigationAreaRuleHandler(object sender, GenericEventArgs<NavigationAreaRules> e)
        {
            Console.WriteLine("Received New Area Rules");

            Console.WriteLine($"Use sixth area for health [{(e.Payload.EnableHealth ? "Enabled" : "Disabled")}]");
            Console.WriteLine($"Fail Safe Mode [[{(e.Payload.FailSafeMode ? "Enabled" : "Disabled")}]");

            foreach (var rule in e.Payload.Rules)
            {
                Console.WriteLine($"Area [{rule.Id}] [{(rule.Enabled ? "Enabled" : "Disabled")}");
                Console.WriteLine($"Break Allowance [{rule.BreakAllowance}]");
                Console.WriteLine($"Allowance Curve Decrement [{rule.AllowanceCurveDecrement}]");
                Console.WriteLine($"Invert Break Logic [{rule.InvertBreakLogic}]");
                Console.WriteLine($"Threshold Delta [{rule.ThresholdDelta}]");
                Console.Write("Points [");
                foreach (var point in rule.AreaPoints)
                {
                    Console.Write($"({point.X}, {point.Y}) ");
                }

                Console.WriteLine("]");
            }
        }


        private static void NavigationAlarmHandler(object sender, GenericEventArgs<NavigationAlarm[]> e)
        {
            Console.Write($"Alarm statuses [");
            foreach (var alarm in e.Payload)
            {
                Console.Write($"{alarm}, ");
            }
            Console.WriteLine("]");
        }


        //Report the client connection status changes
        private static void ConnectionChangedHandler(object sender, ConnectionStateEventArgs connectionStateEventArgs)
        {
            Console.WriteLine("Connection Changed [{0}]", connectionStateEventArgs.State);
        }

        //Handle the configuration message. This is automatically sent after a client connects to the radar.
        //If you recevie the configuration data handler you can assume the client has successfully connected
        //or you can use the conenction chahnged status event
        private static void ConfigurationDataHandler(object sender, GenericEventArgs<TcpConfigurationDataMessage> configurationMessage)
        {
            //Display some of the configuration data received from the radar
            Console.WriteLine("ConfigurationDataHandler - Expected Rotation Rate [{0}Hz]", configurationMessage.Payload.RotationSpeed / 1000.0);
            Console.WriteLine("ConfigurationDataHandler - Range In Bins [{0}]", configurationMessage.Payload.RangeInBins);
            Console.WriteLine("ConfigurationDataHandler - Bin Size [{0}cm]", configurationMessage.Payload.BinSize / 10000.0);
            Console.WriteLine("ConfigurationDataHandler - Range In Metres [{0}m]", configurationMessage.Payload.BinSize / 10000.0 * configurationMessage.Payload.RangeInBins);
            Console.WriteLine("ConfigurationDataHandler - Azimuth Samples [{0}]", configurationMessage.Payload.AzimuthSamples);

            RadarTcpClient.SetNavigationGainAndOffset(1.0f, 0.0f);
            RadarTcpClient.SetNavigationThreshold(60 * 10);

            //Tell the radar to start sending FFT Data
            RadarTcpClient.StartFftData();
            //RadarTcpClient.StartNavigationData();
            _lastRotationReset = DateTime.UtcNow;
        }
    }
}
