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

        virtual bool init(dmxBuffer_t& buffer)                      = 0;
        virtual bool deInit()                                       = 0;
        virtual bool readUSB(usbReadBuffer_t& buffer, size_t& size) = 0;
        virtual bool writeUSB(uint8_t* buffer, size_t size)         = 0;
        virtual void updateBuffer(dmxBuffer_t& buffer)              = 0;
    };

    class WidgetInfo
    {
        public:
        struct fwVersion_t
        {
            uint8_t major = 0;
            uint8_t minor = 0;
        };

        WidgetInfo(uint32_t    serialNr,
                   uint16_t    estaID,
                   uint16_t    deviceID,
                   fwVersion_t fwVersion,
                   const char* manufacturer,
                   const char* deviceName)
            : _serialNr(serialNr)
            , _estaId(estaID)
            , _deviceId(deviceID)
        {
            for (size_t i = 0; i < strlen(manufacturer) && i < 32; i++)
            {
                this->_manufacturer[i] = manufacturer[i];
            }

            for (size_t i = 0; i < strlen(deviceName) && i < 32; i++)
            {
                this->_deviceName[i] = deviceName[i];
            }

            // major version gets upper byte
            // minor version gets lower byte
            this->_fwVersion = (fwVersion.major << 8) | fwVersion.minor;
        }

        WidgetInfo() = default;

        private:
        friend class DMXUSBWidget;
        uint32_t _serialNr         = 0xFFFFFFFF;
        uint16_t _estaId           = 0xFFFF;
        uint16_t _deviceId         = 0x00;
        uint16_t _fwVersion        = 0;
        char     _manufacturer[32] = {};
        char     _deviceName[32]   = {};
    };

    DMXUSBWidget(HWA& hwa);

    bool    init();
    bool    deInit();
    bool    isInitialized();
    void    setWidgetInfo(WidgetInfo&& widgetInfo);
    void    read();
    uint8_t channelValue(uint16_t channel);
    bool    updateChannelValue(uint16_t channel, uint8_t value);

    private:
    HWA& _hwa;

    static constexpr uint8_t START_BYTE = 0x7E;
    static constexpr uint8_t END_BYTE   = 0xE7;

    enum class state_t : uint8_t
    {
        START,
        LABEL,
        LENGTH_LSB,
        LENGTH_MSB,
        DATA,
        END
    };

    enum class label_t : uint8_t
    {
        REPROGRAM_FW_REQ        = 1,
        PROGRAM_FLASH_PAGE      = 2,
        GET_WIDGET_PARAMS       = 3,
        SET_WIDGET_PARAMS       = 4,
        RECEIVED_PACKET         = 5,
        SEND_DMX                = 6,
        SEND_RDM                = 7,
        RECEIVE_DMX_ON_CHANGE   = 8,
        RECEIVED_DMX_ON_CHANGE  = 9,
        GET_SERIAL_NUMBER       = 10,
        SEND_RDM_DISCOVERY_REQ  = 11,
        DEVICE_MANUFACTURER_REQ = 77,
        DEVICE_NAME_REQ         = 78,
        SEND_DIFF_DMX           = 80,
    };

    bool            _initialized     = false;
    state_t         _state           = state_t::START;
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
    WidgetInfo      _widgetInfo;

    void sendHeader(label_t label, size_t size);
    void sendFooter();
    void sendDeviceID();
    void sendDeviceSerNum();
    void sendWidgetParams();
    void sendExtWidgetParams();
    void setNewBuffer();
};