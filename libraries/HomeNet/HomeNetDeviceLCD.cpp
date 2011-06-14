/*
 * Copyright (c) 2011 Matthew Doll <mdoll at homenet.me>.
 *
 * This file is part of HomeNet.
 *
 * HomeNet is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * HomeNet is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with HomeNet.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <HomeNetDevices.h>
#include "HomeNetDeviceLCD.h"
#include <WProgram.h>

void HomeNetDeviceLCD::init() {
#ifdef DEBUG
    Serial.println("Init LCD");
#endif
    HomeNetDevicePortI2C::init();
    _displayCount = 0;

    _lcd = new LiquidCrystalI2C(*_port);
    _lcd->begin(16, 2);
    _lcd->setCursor(0, 0);
    _lcd->print("LCD at ");
    _lcd->print(_location, DEC);
    delay(1000);
    _lcd->clear();
}

HomeNetPartialPacket* HomeNetDeviceLCD::process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload) {
#ifdef DEBUG
    Serial.println("Process LCD");
#endif

    //receive outside packets
  //  if (fromNode != _homeNet->_nodeId) {
        for (uint8_t i = 0; i < _displayCount; i++) {
            if ((fromNode == _display[i].fromNode) && (fromDevice == _display[i].fromDevice)) {
#ifdef DEBUG
                    Serial.println("LCD Display Packet Matched");
#endif
                 _lcd->setCursor(0, _display[i].row);
                 char line[16];// = "                ";
                 char num[9];
                //display = &_display[i];
                switch (command) {

                    case CMD_REPLYBYTE:
                        //_lcd->print("Byte");
                        //_lcd->print(payload.getByte(),DEC);
                        sprintf(line,_display[i].message, itoa(payload.getByte(), num, 10));
                        break;

                    case CMD_REPLYINT:
                        //_lcd->print("Int");
                        //_lcd->print(payload.getInt(),DEC);
                        sprintf(line,_display[i].message, itoa(payload.getInt(), num, 10));
                        break;

                    case CMD_REPLYFLOAT:
                        //_lcd->print(_display[i].message);
                        //_lcd->print();
                        sprintf(line,_display[i].message, dtostrf(payload.getFloat(), 5, 2,num));
                        break;
                    
                }
                _lcd->print(line);
                return NULL;
            } 
        }
    //}

    switch (command) {

        case CMD_SETVALUE:
        case CMD_SETSTRING:
            uint8_t length;
            length = payload.getLength();
            _lcd->clear();
            _lcd->setCursor(0, 0);
            if (length <= 16) {
                for (uint8_t i = 0; i < length; i++) {
                    _lcd->print(payload.getByte(i));
                }
                break;
            }
            for (uint8_t i = 0; i < 16; i++) {
                _lcd->print(payload.getByte(i));
            }
            _lcd->setCursor(0, 1);
            for (uint8_t i = 16; i < length; i++) {
                _lcd->print(payload.getByte(i));
            }
            break;

        case CMD_ON:
            _lcd->backlight();
            break;

        case CMD_OFF:
            _lcd->noBacklight();
            break;
    }
    return NULL;
}

void HomeNetDeviceLCD::registerDisplay(Display display[], uint8_t count){
    _displayCount = count;
    _display = display;
}