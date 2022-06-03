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
{
    _activeBuffer = &_buffer1;
    _writeBuffer  = &_buffer2;
}

bool DMXUSBWidget::init()
{
    if (_initialized)
    {
        return true;
    }

    if (!_hwa.init(*_activeBuffer))
    {
        return false;
    }

    _initialized = true;
    _state       = state_t::START;
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

void DMXUSBWidget::setWidgetInfo(WidgetInfo&& widgetInfo)
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
        bool updateBuffer = false;

        for (size_t i = 0; i < size; i++)
        {
            uint8_t data = _usbReadBuffer[i];

            switch (_state)
            {
            case state_t::START:
            {
                if (data == START_BYTE)
                {
                    _state = state_t::LABEL;
                }
            }
            break;

            case state_t::LABEL:
            {
                _label = data;

                if ((_label == static_cast<uint8_t>(label_t::SEND_DMX)) || (_label == static_cast<uint8_t>(label_t::SEND_DIFF_DMX)))
                {
                    _state = state_t::LENGTH_LSB;

                    if (_label == static_cast<uint8_t>(label_t::SEND_DIFF_DMX))
                    {
                        _diffMode = true;
                    }
                }
                else
                {
                    _state = state_t::END;
                }
            }
            break;

            case state_t::LENGTH_LSB:
            {
                _dataLength = data;
                _state      = state_t::LENGTH_MSB;
            }
            break;

            case state_t::LENGTH_MSB:
            {
                _dataLength |= (data << 8);
                _dataCounter = 0;
                _state       = (_dataLength > 0) ? state_t::DATA : state_t::END;
            }
            break;

            case state_t::DATA:
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
                        _byteParseCount                    = 0;
                        _writeBuffer->at(_channelToUpdate) = data;
                        updateBuffer                       = true;
                    }
                    break;

                    default:
                        break;
                    }
                }
                else
                {
                    // normal mode
                    _writeBuffer->at(_dataCounter) = data;
                    updateBuffer                   = true;
                }

                if (++_dataCounter == _dataLength)
                {
                    _state = state_t::END;
                }
            }
            break;

            case state_t::END:
            {
                if (data == END_BYTE)
                {
                    _state         = state_t::START;
                    auto labelEnum = static_cast<label_t>(_label);

                    switch (labelEnum)
                    {
                    case label_t::SEND_DMX:
                    {
                        // only in normal/full mode
                        updateBuffer = true;
                    }
                    break;

                    case label_t::GET_SERIAL_NUMBER:
                    {
                        constexpr size_t SIZE = 4;

                        uint8_t buffer[SIZE] = {
                            static_cast<uint8_t>((_widgetInfo._serialNr >> 0) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo._serialNr >> 8) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo._serialNr >> 16) & 0xFF),
                            static_cast<uint8_t>((_widgetInfo._serialNr >> 24) & 0xFF),
                        };

                        sendHeader(labelEnum, SIZE);
                        _hwa.writeUSB(buffer, SIZE);
                        sendFooter();
                    }
                    break;

                    case label_t::GET_WIDGET_PARAMS:
                    {
                        constexpr size_t SIZE = 5;

                        uint8_t buffer[SIZE] = {
                            static_cast<uint8_t>((_widgetInfo._fwVersion >> 0) & 0xFF),    // firmware version LSB
                            static_cast<uint8_t>((_widgetInfo._fwVersion >> 8) & 0xFF),    // firmware version MSB
                            0x09,                                                          // DMX output break time in 10.67 microsecond units: 9
                            0x02,                                                          // DMX output Mark After Break time in 10.67 microsecond units: 1
                            0x28,                                                          // DMX output rate in packets per second: 40
                        };

                        sendHeader(labelEnum, SIZE);
                        _hwa.writeUSB(buffer, SIZE);
                        sendFooter();
                    }
                    break;

                    case label_t::DEVICE_MANUFACTURER_REQ:
                    {
                        size_t size = sizeof(_widgetInfo._estaId) + strlen(_widgetInfo._manufacturer);

                        uint8_t buffer[32];

                        buffer[0] = _widgetInfo._estaId & 0xFF;
                        buffer[1] = _widgetInfo._estaId >> 8 & 0xFF;

                        for (size_t i = 0; i < strlen(_widgetInfo._manufacturer); i++)
                        {
                            buffer[2 + i] = _widgetInfo._manufacturer[i];
                        }

                        sendHeader(labelEnum, size);
                        _hwa.writeUSB(buffer, size);
                        sendFooter();
                    }
                    break;

                    case label_t::DEVICE_NAME_REQ:
                    {
                        size_t size = sizeof(_widgetInfo._deviceId) + strlen(_widgetInfo._deviceName);

                        uint8_t buffer[32];

                        buffer[0] = _widgetInfo._deviceId & 0xFF;
                        buffer[1] = _widgetInfo._deviceId >> 8 & 0xFF;

                        for (size_t i = 0; i < strlen(_widgetInfo._deviceName); i++)
                        {
                            buffer[2 + i] = _widgetInfo._deviceName[i];
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
                _state = state_t::START;
            }
            break;
            }
        }

        if (updateBuffer)
        {
            setNewBuffer();
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

void DMXUSBWidget::setNewBuffer()
{
    std::swap(_writeBuffer, _activeBuffer);
    _hwa.updateBuffer(*_activeBuffer);

    std::copy(std::begin(*_activeBuffer), std::end(*_activeBuffer), std::begin(*_writeBuffer));
}

uint8_t DMXUSBWidget::channelValue(uint16_t channel)
{
    return _writeBuffer->at(channel);
}

bool DMXUSBWidget::updateChannelValue(uint16_t channel, uint8_t value)
{
    _writeBuffer->at(channel) = value;
    setNewBuffer();

    return true;
}
