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
#include <string.h>
#include <array>

class DMXUSBWidget
{
    public:
    static constexpr size_t USB_READ_BUFFER_SIZE = 64;

    using dmxBuffer_t     = std::array<uint8_t, 513>;
    using usbReadBuffer_t = std::array<uint8_t, USB_READ_BUFFER_SIZE>;

    class HWA
    {
        public:
        HWA() = default;

        virtual bool init()                                         = 0;
        virtual bool deInit()                                       = 0;
        virtual bool readUSB(usbReadBuffer_t& buffer, size_t& size) = 0;
        virtual bool writeUSB(uint8_t* buffer, size_t size)         = 0;
        virtual void setBuffer(dmxBuffer_t& buffer)                 = 0;
    };

    struct widgetInfo_t
    {
        uint32_t serialNr         = 0xFFFFFFFF;
        uint16_t estaID           = 0xFFFF;
        uint16_t deviceID         = 0x00;
        uint16_t fwVersion        = 0;
        char     manufacturer[32] = {};
        char     deviceName[32]   = {};

        struct fwVersion_t
        {
            uint8_t major = 0;
            uint8_t minor = 0;
        };

        widgetInfo_t(uint32_t    serialNr,
                     uint16_t    estaID,
                     uint16_t    deviceID,
                     fwVersion_t fwVersion,
                     const char* manufacturer,
                     const char* deviceName)
            : serialNr(serialNr)
            , estaID(estaID)
            , deviceID(deviceID)
        {
            for (size_t i = 0; i < strlen(manufacturer) && i < 32; i++)
            {
                this->manufacturer[i] = manufacturer[i];
            }

            for (size_t i = 0; i < strlen(deviceName) && i < 32; i++)
            {
                this->deviceName[i] = deviceName[i];
            }

            // major version gets upper byte
            // minor version gets lower byte
            this->fwVersion = (fwVersion.major << 8) | fwVersion.minor;
        }

        widgetInfo_t() = default;
    };

    DMXUSBWidget(HWA& hwa);

    bool    init();
    bool    deInit();
    bool    isInitialized();
    void    setWidgetInfo(widgetInfo_t&& widgetInfo);
    void    read();
    uint8_t channelValue(uint16_t channel);
    bool    updateChannelValue(uint16_t channel, uint8_t value);

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
        reprogramFWreq        = 1,
        programFlashPage      = 2,
        getWidgetParams       = 3,
        setWidgetParams       = 4,
        receivedPacket        = 5,
        sendDMX               = 6,
        sendRDM               = 7,
        receiveDMXonChange    = 8,
        receivedDMXonChange   = 9,
        getSerialNumber       = 10,
        sendRDMdiscoveryReq   = 11,
        deviceManufacturerReq = 77,
        deviceNameReq         = 78,
        sendDiffDMX           = 80,
    };

    bool            _initialized     = false;
    state_t         _state           = state_t::start;
    uint8_t         _label           = 0;
    uint16_t        _dataLength      = 0;
    uint16_t        _dataCounter     = 0;
    usbReadBuffer_t _usbReadBuffer   = {};
    uint16_t        _channelToUpdate = 0;
    uint8_t         _byteParseCount  = 0;
    bool            _diffMode        = false;
    dmxBuffer_t     _buffer1         = {};
    dmxBuffer_t     _buffer2         = {};
    dmxBuffer_t*    _activeBuffer    = nullptr;
    dmxBuffer_t*    _writeBuffer     = nullptr;
    widgetInfo_t    _widgetInfo;

    void sendHeader(label_t label, size_t size);
    void sendFooter();
    void sendDeviceID();
    void sendDeviceSerNum();
    void sendWidgetParams();
    void sendExtWidgetParams();
    void setNewBuffer();
};