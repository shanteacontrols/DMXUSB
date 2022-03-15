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

#include "DMXUSBWidget.h"
#include <utility>

DMXUSBWidget::DMXUSBWidget(HWA& hwa)
    : _hwa(hwa)
{}

bool DMXUSBWidget::init()
{
    if (_initialized)
    {
        return true;
    }

    if (!_hwa.init())
    {
        return false;
    }

    _initialized = true;
    _state       = state_t::start;
    _dataLength  = 0;
    _dataCounter = 0;

    return true;
}

bool DMXUSBWidget::deInit()
{
    if (_initialized)
    {
        _initialized = false;
        return _hwa.deInit();
    }

    return true;
}

bool DMXUSBWidget::isInitialized()
{
    return _initialized;
}

void DMXUSBWidget::setWidgetInfo(widgetInfo_t&& widgetInfo)
{
    _widgetInfo = std::move(widgetInfo);
}

void DMXUSBWidget::read()
{
    if (!_initialized)
    {
        return;
    }

    size_t size = 0;

    if (_hwa.readUSB(_usbReadBuffer, size))
    {
        for (size_t i = 0; i < size; i++)
        {
            uint8_t data = _usbReadBuffer[i];

            switch (_state)
            {
            case state_t::start:
            {
                if (data == START_BYTE)
                {
                    _state = state_t::label;
                }
            }
            break;

            case state_t::label:
            {
                _label = data;

                if ((_label == static_cast<uint8_t>(label_t::sendDMX)) || (_label == static_cast<uint8_t>(label_t::sendDiffDMX)))
                {
                    _state = state_t::length_lsb;

                    if (_label == static_cast<uint8_t>(label_t::sendDiffDMX))
                    {
                        _diffMode = true;
                    }
                }
                else
                {
                    _state = state_t::end;
                }
            }
            break;

            case state_t::length_lsb:
            {
                _dataLength = data;
                _state      = state_t::length_msb;
            }
            break;

            case state_t::length_msb:
            {
                _dataLength |= (data << 8);
                _dataCounter = 0;
                _state       = (_dataLength > 0) ? state_t::data : state_t::end;
            }
            break;

            case state_t::data:
            {
                if (_diffMode)
                {
                    // diff mode:
                    // a packet is composed of 2 bytes of channel to update
                    // next value is actual channel value
                    // the pattern repeats for requested number of channels
                    _byteParseCount++;

                    switch (_byteParseCount)
                    {
                    case 1:
                    {
                        _channelToUpdate = data;
                    }
                    break;

                    case 2:
                    {
                        _channelToUpdate |= (data << 8);
                    }
                    break;

                    case 3:
                    {
                        _byteParseCount = 0;
                        _hwa.updateChannel(_channelToUpdate, data);
                    }
                    break;

                    default:
                        break;
                    }
                }
                else
                {
                    // normal mode
                    _hwa.updateChannel(_dataCounter, data);
                }

                if (++_dataCounter == _dataLength)
                {
                    _state = state_t::end;
                }
            }
            break;

            case state_t::end:
            {
                if (data == END_BYTE)
                {
                    _state         = state_t::start;
                    auto labelEnum = static_cast<label_t>(_label);

                    switch (labelEnum)
                    {
                    case label_t::sendDMX:
                    {
                        _hwa.packetComplete();
                    }
                    break;

                    case label_t::getSerialNumber:
                    {
                        constexpr size_t size = 4;

                        uint8_t buffer[size] = {
                            static_cast<uint8_t>((_widgetInfo.serialNr >> 0) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo.serialNr >> 8) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo.serialNr >> 16) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo.serialNr >> 24) & 0xFF),
                        };

                        sendHeader(labelEnum, size);
                        _hwa.writeUSB(buffer, size);
                        sendFooter();
                    }
                    break;

                    case label_t::getWidgetParams:
                    {
                        constexpr size_t size = 5;

                        uint8_t buffer[size] = {
                            static_cast<uint8_t>((_widgetInfo.fwVersion >> 0) & 0xFF),    // firmware version LSB
                            static_cast<uint8_t>((_widgetInfo.fwVersion >> 8) & 0xFF),    // firmware version MSB
                            0x09,                                                         // DMX output break time in 10.67 microsecond units: 9
                            0x02,                                                         // DMX output Mark After Break time in 10.67 microsecond units: 1
                            0x28,                                                         // DMX output rate in packets per second: 40
                        };

                        sendHeader(labelEnum, size);
                        _hwa.writeUSB(buffer, size);
                        sendFooter();
                    }
                    break;

                    case label_t::deviceManufacturerReq:
                    {
                        size_t size = sizeof(_widgetInfo.estaID) + strlen(_widgetInfo.manufacturer);

                        uint8_t buffer[32];

                        buffer[0] = _widgetInfo.estaID & 0xFF;
                        buffer[1] = _widgetInfo.estaID >> 8 & 0xFF;

                        for (size_t i = 0; i < strlen(_widgetInfo.manufacturer); i++)
                        {
                            buffer[2 + i] = _widgetInfo.manufacturer[i];
                        }

                        sendHeader(labelEnum, size);
                        _hwa.writeUSB(buffer, size);
                        sendFooter();
                    }
                    break;

                    case label_t::deviceNameReq:
                    {
                        size_t size = sizeof(_widgetInfo.deviceID) + strlen(_widgetInfo.deviceName);

                        uint8_t buffer[32];

                        buffer[0] = _widgetInfo.deviceID & 0xFF;
                        buffer[1] = _widgetInfo.deviceID >> 8 & 0xFF;

                        for (size_t i = 0; i < strlen(_widgetInfo.deviceName); i++)
                        {
                            buffer[2 + i] = _widgetInfo.deviceName[i];
                        }

                        sendHeader(labelEnum, size);
                        _hwa.writeUSB(buffer, size);
                        sendFooter();
                    }
                    break;

                    default:
                        break;
                    }
                }

                _channelToUpdate = 0;
                _byteParseCount  = 0;
                _diffMode        = false;
            }
            break;

            default:
            {
                _state = state_t::start;
            }
            break;
            }
        }
    }
}

void DMXUSBWidget::sendHeader(label_t label, size_t size)
{
    uint8_t buffer[4];

    buffer[0] = START_BYTE;
    buffer[1] = static_cast<uint8_t>(label);
    buffer[2] = size & 0xFF;
    buffer[3] = (size >> 8) & 0xFF;

    _hwa.writeUSB(buffer, 4);
}

void DMXUSBWidget::sendFooter()
{
    uint8_t footer = END_BYTE;
    _hwa.writeUSB(&footer, 1);
}