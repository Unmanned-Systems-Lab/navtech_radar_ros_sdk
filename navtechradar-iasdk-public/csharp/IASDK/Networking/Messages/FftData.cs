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

using Navtech.IASDK.Interfaces;

namespace Navtech.IASDK.Networking.Messages
{
    /// <summary>
    /// Represents an FFT Data message and implements the <see cref="IRawData"/> interface
    /// Includes both the raw data and the class represents of the message
    /// </summary>
    public class FftData : IRawData
    {
        /// <summary>
        /// Represents network data as a TCP FFT Data Message <see cref="TcpFftDataMessage"/>
        /// </summary>
        public TcpFftDataMessage Message { get; private set; }

        /// <summary>
        /// The raw network message bytes 
        /// </summary>
        public byte[] Data { get; private set; }

        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="message">TCP FFT Message</param>
        /// <param name="data">Message as raw bytes</param>
        public FftData(TcpFftDataMessage message, byte[] data)
        {
            Message = message;
            Data = data;
        }
    }
}
