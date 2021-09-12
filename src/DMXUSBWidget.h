/*

Copyright 2021 Igor Petrovic

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

*/

#pragma once

#include <cstddef>
#include <inttypes.h>

class DMXUSBWidget
{
    public:
    class HWA
    {
        public:
        HWA() = default;

        virtual bool init()                                                       = 0;
        virtual bool deInit()                                                     = 0;
        virtual bool readUSB(uint8_t* buffer, size_t& size, const size_t maxSize) = 0;
        virtual bool writeUSB(uint8_t* buffer, size_t size)                       = 0;
        virtual bool updateChannel(uint16_t channel, uint8_t value)               = 0;
        virtual void packetComplete()                                             = 0;
    };

    DMXUSBWidget(HWA& hwa)
        : _hwa(hwa)
    {}

    bool init();
    bool deInit();
    bool isInitialized();
    void setSerialNumber(uint32_t serialNr);
    void read();

    private:
    HWA& _hwa;

    static constexpr uint8_t START_BYTE = 0x7E;
    static constexpr uint8_t END_BYTE   = 0xE7;

    enum class state_t : uint8_t
    {
        start,
        label,
        length_lsb,
        length_msb,
        data,
        end
    };

    enum class label_t : uint8_t
    {
        reprogramFWreq      = 1,
        programFlashPage    = 2,
        getWidgetParams     = 3,
        setWidgetParams     = 4,
        receivedPacket      = 5,
        sendDMX             = 6,
        sendRDM             = 7,
        receiveDMXonChange  = 8,
        receivedDMXonChange = 9,
        getSerialNumber     = 10,
        sendRDMdiscoveryReq = 11,
    };

    static constexpr size_t USB_BUFFER_SIZE = 64;

    bool     _initialized                    = false;
    uint32_t _serialNr                       = 0xFFFFFFFF;
    state_t  _state                          = state_t::start;
    uint8_t  _label                          = 0;
    uint16_t _dataLength                     = 0;
    uint16_t _dataCounter                    = 0;
    uint16_t _channelToUpdate                = 0;
    uint8_t  _byteParseCount                 = 0;
    uint8_t  _usbReadBuffer[USB_BUFFER_SIZE] = {};

    void sendDeviceID();
    void sendDeviceSerNum();
    void sendWidgetParams();
    void sendExtWidgetParams();
};