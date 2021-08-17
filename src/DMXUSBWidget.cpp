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

bool DMXUSBWidget::init()
{
    if (_initialized)
        return true;

    if (!_hwa.init())
        return false;

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

void DMXUSBWidget::setSerialNumber(uint32_t serialNr)
{
    _serialNr = serialNr;
}

void DMXUSBWidget::read()
{
    if (!_initialized)
        return;

    size_t size = 0;

    if (_hwa.readUSB(_usbReadBuffer, size, USB_BUFFER_SIZE))
    {
        for (size_t i = 0; i < size; i++)
        {
            uint8_t data = _usbReadBuffer[i];

            switch (_state)
            {
            case state_t::start:
            {
                if (data == START_BYTE)
                    _state = state_t::label;
            }
            break;

            case state_t::label:
            {
                _label = data;

                if (_label == static_cast<uint8_t>(label_t::sendDMX))
                {
                    _state = state_t::length_lsb;
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
                _hwa.updateChannel(_dataCounter, data);

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
                    case label_t::getSerialNumber:
                    {
                        uint8_t buffer[9] = {
                            START_BYTE,
                            _label,
                            4,    // data length LSB
                            0,    // data length MSB
                            static_cast<uint8_t>((_serialNr >> 24) & 0xFF),
                            static_cast<uint8_t>((_serialNr >> 16) & 0xFF),
                            static_cast<uint8_t>((_serialNr >> 8) & 0xFF),
                            static_cast<uint8_t>((_serialNr >> 0) & 0xFF),
                            END_BYTE
                        };

                        _hwa.writeUSB(buffer, 9);
                    }
                    break;

                    case label_t::getWidgetParams:
                    {
                        uint8_t buffer[10] = {
                            START_BYTE,
                            _label,
                            0x05,    // data length LSB
                            0x00,    // data length MSB
                            0x00,    // firmware version LSB
                            0x01,    // firmware version MSB
                            0x09,    // DMX output break time in 10.67 microsecond units: 9
                            0x01,    // DMX output Mark After Break time in 10.67 microsecond units: 1
                            0x28,    // DMX output rate in packets per second: 40
                            END_BYTE
                        };

                        _hwa.writeUSB(buffer, 10);
                    }
                    break;

                    default:
                        break;
                    }
                }
            }
            break;

            default:
                _state = state_t::start;
                break;
            }
        }
    }
}