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

using Navtech.IASDK.Enums;
using Navtech.IASDK.Events;
using Navtech.IASDK.Extensions;
using Navtech.IASDK.Interfaces;
using Navtech.IASDK.Networking.Messages;
using Navtech.IASDK.Utility;
using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace Navtech.IASDK.Networking;

/// <summary>
/// Represents a TCP client with specific knowledge
/// of the radar's Colossus network protocol. This class
/// encapsulates the class <see cref="TcpClient{T,TU}"/>
/// </summary>
public class RadarTcpClient
{
    /// <summary>
    /// Informs subscribers when the client receives a configuration message
    /// </summary>
    public event EventHandler<GenericEventArgs<TcpConfigurationDataMessage>> OnConfigurationData;
    /// <summary>
    /// Informs subscribers when the client receives an FFT Data message
    /// </summary>
    public event EventHandler<GenericEventArgs<FftData>> OnFftData;
    /// <summary>
    /// Informs subscribers when the client receives a Navigation message
    /// </summary>
    public event EventHandler<NavigationDataEventArgs> OnNavigationData;

    /// <summary>
    /// Informs subscribers when the client receives an Area Rules message
    /// </summary>
    public event EventHandler<GenericEventArgs<NavigationAreaRules>> OnNavigationAreaRules;

    /// <summary>
    /// Informs subscribers when the client receives Navigation Alarms
    /// </summary>
    public EventHandler<GenericEventArgs<NavigationAlarm[]>> OnNavigationAlarm;

    /// <summary>
    /// Informs subscribers when the client receives accelerometer data
    /// </summary>
    public EventHandler<GenericEventArgs<AccelerometerData>> OnAccelerometerData;

    /// <summary>
    /// Informs subscribers when the client's connection status changes
    /// </summary>
    public event EventHandler<ConnectionStateEventArgs> OnConnectionChanged;

    private readonly TcpClient<RadarTcpClientConnection, TcpNetworkDataMessage> _radarClient;

    private ConnectionState _requiredConnectionState;
    private ConnectionState _currentConnectionState;
    private string _ipAddress;
    private ushort _port;
    private ushort _binSize;

    /// <summary>
    /// Default constructor
    /// </summary>
    public RadarTcpClient()
    {
        _radarClient = new TcpClient<RadarTcpClientConnection, TcpNetworkDataMessage>();
        _radarClient.OnData += OnDataHandler;
        _radarClient.OnConnectionChanged += OnConnectionChangedHandler;
    }

    /// <summary>
    /// Attempts to connect the client to a radar on 
    /// the supplied IP address and port
    /// </summary>
    /// <param name="ipAddress">IP address of the radar</param>
    /// <param name="port">Radar TCP Port - leave blank to use default</param>
    public void Connect(string ipAddress, ushort port = 6317)
    {
        _ipAddress = ipAddress;
        _port = port;
        _requiredConnectionState = ConnectionState.Connected;
        _radarClient.Connect(_ipAddress, _port);
    }

    /// <summary>
    /// Disconnects the client from the radar
    /// </summary>
    public void Disconnect()
    {
        _requiredConnectionState = ConnectionState.Disconnected;
        _radarClient.Disconnect();
        OnConnectionChanged?.Invoke(this, new ConnectionStateEventArgs(ConnectionState.Disconnected));
    }

    /// <summary>
    /// Sends a compatible raw data message to the radar
    /// <see cref="IRawData"/>
    /// </summary>
    /// <param name="data">Compatiable raw data message</param>
    public void SendData(IRawData data)
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        _radarClient.SendData(data);
    }

    /// <summary>
    /// Sends a "Start FFT Data" command to the connected radar
    /// This instructs the radar to start issuing FFT Data messages
    /// </summary>
    public void StartFftData()
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.StartFftData));
    }

    /// <summary>
    /// Sends a "Stop FFT Data" command to the connected radar
    /// This stops the radar sending FFT Data messages
    /// </summary>
    public void StopFftData()
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.StopFftData));
    }

    /// <summary>
    /// Sends a "Start Navigation" command to the connected radar
    /// This instructs the radar to start performing peak extraction and
    /// sending navigation data messages
    /// </summary>
    public void StartNavigationData()
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.StartNavData));
    }

    /// <summary>
    /// Sends a "Stop Navigation" command to the connected radar
    /// </summary>
    public void StopNavigationData()
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.StopNavData));
    }

    /// <summary>
    /// Sets the peak extraction threshold for Navigation data. Signal peaks above
    /// this threshold will be sub-resolved for improved range resolution and reported
    /// </summary>
    /// <param name="threshold">The power threshold to set</param>
    public void SetNavigationThreshold(ushort threshold)
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        var buffer = BitConverter.GetBytes(NetConversions.SwapUInt16(threshold));
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.SetNavThreshold, buffer));
    }

    /// <summary>
    /// Sets the range gain and offset for this radar. Typically this is done 
    /// during factory calibration but these values can be changed if further
    /// calibration is required
    /// </summary>
    /// <param name="gain">Range gain (multiplier) to be applied to all range measurements</param>
    /// <param name="offset">Range offset to be applied to all range measurements</param>
    public void SetNavigationGainAndOffset(float gain, float offset)
    {
        if (_currentConnectionState != ConnectionState.Connected) return;
        var buffer = new byte[8];
        Buffer.BlockCopy(BitConverter.GetBytes(NetConversions.SwapUInt32((uint)(gain * 1000000))), 0, buffer, 0, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(NetConversions.SwapUInt32((uint)(offset * 1000000))), 0, buffer, 4, 4);
        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.SetNavRangeOffsetAndGain, buffer));
    }

    /// <summary>
    /// Updates the contour map on the radar. A contour map restricts
    /// the radar data to a specific range in each azimuth. This can significantly
    /// reduce the network bandwidth if you do not need to see all the radar's data
    /// </summary>
    /// <param name="contourData">List of ranges to limit the data for each azimituh. Should include one value for each degree (360)</param>
    public void UpdateContourMap(List<ushort> contourData)
    {
        if (contourData.Count != 360 || _currentConnectionState != ConnectionState.Connected) return;
        var resolution = _binSize / 10000.0;
        var contour = new List<byte>();
        for (var i = 0; i < 360; i++)
        {
            var result = (ushort)Math.Ceiling(contourData[i] / resolution);
            contour.Add((byte)((result & 0xff00) >> 8));
            contour.Add((byte)(result & 0x00ff));
        }

        _radarClient.SendData(new TcpNetworkDataMessage(TcpNetworkDataMessageType.SetNavRangeOffsetAndGain, contour.ToArray()));
    }

    private void OnConnectionChangedHandler(object sender, ConnectionStateEventArgs e)
    {
        if (_requiredConnectionState != ConnectionState.Connected) return;

        _currentConnectionState = e.State;
        switch (_currentConnectionState)
        {
            case ConnectionState.Connected:
                OnConnectionChanged?.Invoke(this, e);
                Debug.WriteLine($"TrackerTCPClient - Connected to [{_ipAddress}:{_port}]");
                break;
            case ConnectionState.Disconnected:
                OnConnectionChanged?.Invoke(this, e);
                Debug.WriteLine($"TrackerTCPClient - Retrying connection to [{_ipAddress}:{_port}]");
                _radarClient.Connect(_ipAddress, _port);
                break;
        }
    }

    private void OnDataHandler(object sender, GenericEventArgs<TcpNetworkDataMessage> e)
    {
        switch (e.Payload.MessageType)
        {
            case TcpNetworkDataMessageType.Configuration:
                HandleConfigurationMessage(e.Payload);
                break;
            case TcpNetworkDataMessageType.FftData:
                HandleFftMessage(e.Payload);
                break;
            case TcpNetworkDataMessageType.NavigationData:
                HandleNavigationMessage(e.Payload);
                break;
            case TcpNetworkDataMessageType.NavigationAreaRules:
                HandleAreaRules(e.Payload);
                break;
            case TcpNetworkDataMessageType.NavigationAlarmData:
                HandleNavigationAlarms(e.Payload);
                break;
            case TcpNetworkDataMessageType.AccelerometerData:
                HandleAccelerometerData(e.Payload);
                break;
        }
    }

    private void HandleFftMessage(TcpNetworkDataMessage message)
    {
        var fftDataBuffer = new byte[message.PayloadSize - TcpFftDataMessage.Size];
        Buffer.BlockCopy(message.Payload, TcpFftDataMessage.Size, fftDataBuffer, 0, message.PayloadSize - TcpFftDataMessage.Size);
        var fftMessage = message.Payload.MarshalToObject<TcpFftDataMessage>();
        OnFftData?.Invoke(this, new GenericEventArgs<FftData> { Payload = new FftData(fftMessage, fftDataBuffer) });
    }

    private void HandleConfigurationMessage(TcpNetworkDataMessage message)
    {
        var configDataBuffer = new byte[message.PayloadSize - TcpConfigurationDataMessage.Size];
        Buffer.BlockCopy(message.Payload, TcpConfigurationDataMessage.Size, configDataBuffer, 0, message.PayloadSize - TcpConfigurationDataMessage.Size);
        var configMessage = message.Payload.MarshalToObject<TcpConfigurationDataMessage>();
        _binSize = configMessage.BinSize;
        OnConfigurationData?.Invoke(this, new GenericEventArgs<TcpConfigurationDataMessage> { Payload = configMessage });
    }

    private void HandleNavigationMessage(TcpNetworkDataMessage message)
    {
        var navDataBuffer = new byte[message.PayloadSize - TcpNavigationDataMessage.Size];
        Buffer.BlockCopy(message.Payload, TcpNavigationDataMessage.Size, navDataBuffer, 0, message.PayloadSize - TcpNavigationDataMessage.Size);
        var navMessage = message.Payload.MarshalToObject<TcpNavigationDataMessage>();
        var navigationMessage = new NavigationData(navMessage);
        for (var i = 0; i < navDataBuffer.Length; i += 6)
        {
            var navReturn = new NavigationReturn
            {
                Range = NetConversions.SwapUInt32(BitConverter.ToUInt32(navDataBuffer.Slice(i, i + 4), 0)) / 1000000.0,
                Power = NetConversions.SwapUInt16(BitConverter.ToUInt16(navDataBuffer.Slice(i + 4, i + 6), 0))
            };
            navigationMessage.NavigationReturns.Add(navReturn);
        }
        OnNavigationData?.Invoke(this, new NavigationDataEventArgs(navigationMessage));
    }

    private void HandleAreaRules(TcpNetworkDataMessage message)
    {
        var areaRuleBuffer = new byte[message.PayloadSize - TcpAreaRuleMessage.Size];
        Buffer.BlockCopy(message.Payload, TcpAreaRuleMessage.Size, areaRuleBuffer, 0, message.PayloadSize - TcpNavigationDataMessage.Size);
        var areaRuleMessage = message.Payload.MarshalToObject<TcpAreaRulesMessage>();
        OnNavigationAreaRules?.Invoke(this, new () { Payload = new NavigationAreaRules(areaRuleMessage, areaRuleBuffer) });
    }

    private void HandleNavigationAlarms(TcpNetworkDataMessage message)
    {
        var alarms = new NavigationAlarm[message.Payload.Length];
        for (var i = 0; i < message.Payload.Length; i++)
        {
            alarms[i] = new NavigationAlarm(i + 1, message.Payload[i] != 0 ? AlarmState.Active : AlarmState.Inactive);
        }
        OnNavigationAlarm?.Invoke(this, new () { Payload = alarms });
    }

    private void HandleAccelerometerData(TcpNetworkDataMessage message)
    {
        var accelerometerDataMessage = message.Payload.MarshalToObject<AccelerometerDataMessage>();
        OnAccelerometerData.Invoke(this, new GenericEventArgs<AccelerometerData> { Payload = new AccelerometerData(accelerometerDataMessage) });
    }
}