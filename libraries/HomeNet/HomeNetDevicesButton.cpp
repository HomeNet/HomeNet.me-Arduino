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

#include "HomeNet.h"
#include "HomeNetDevices.h"
#include "HomeNetDevicesButton.h"
#include <WProgram.h>

//5 Button//////////////////////////////////////////////////////////////////////

void HomeNetDevice5Button::init() {
#ifdef DEBUG
    Serial.println("Init 5 Button");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
    HomeNetDeviceI2C::init();

    // Serial.print("Config: ");
    // Serial.println(getByte(IOCON), BIN);
    setByte(IODIR, B11111111); //set everything to inputs
    setByte(GPINTEN, B00111111); //set interrupts
    setByte(DEFVAL, B00011111); //set default value for buttons
    setByte(INTCON, B00000000); //compare buttons 1-5 to normal state, 6 switch compare to the change
    setByte(GPPU, B00111111); //set pullup resistors on the switches
    //Serial.println("Set LED");
    setLED(1);
    delay(500);
    setLED(2);
    delay(500);
    setLED(3);
    delay(500);
    setLED(4);
    delay(500);
    setLED(0);


    //get Enabled status

    lastButton = B11111111;

    byte button = getByte(GPIO);

    if (bitRead(button, 5) == 0) {
        buttonsEnabled = true;

#ifdef DEBUG
        Serial.println("Buttons Enabled");
#endif
    } else {
        bitClear(lastButton,5);
#ifdef DEBUG
        Serial.println("Buttons Disabled");
#endif
        buttonsEnabled = false;
    }

    delay(1000);
    
    attachInterrupt(1, onInterrupt, FALLING); //Setup interrupt
    //setup the interrupt the same
    _port->mode3(INPUT);
    // _port->digiWrite3(HIGH);
   // Serial.print("Int Value: ");
   // Serial.println(_port->digiRead3(), DEC);

}

HomeNetPartialPacket* HomeNetDevice5Button::interrupt(const uint8_t& command, const HomeNetPayload& payload) {
    byte button = getButton();

    if (button != 0) {

        for(byte i = 0; i < _buttonCount; i++){
            if(_buttons[i].button == button){
                _homeNet->addUdpPacket(_location,_buttons[i].toNode, _buttons[i].toDevice, _buttons[i].command,_buttons[i].payload);
            }
        }


        // return partialPacket(_location, CMD_REPLYBYTE, buffer.payload());
    }
    return NULL;
}

HomeNetPartialPacket* HomeNetDevice5Button::process(const uint8_t& command, const HomeNetPayload& payload) {
    switch (command) {

        case CMD_GETBYTE:
        case CMD_GETVALUE:
        {

            // return partialPacket(_location, CMD_REPLYBYTE, buffer.payload());
            break;
        }
        case CMD_REPLYBYTE:
        case CMD_SETBYTE:
        case CMD_SETVALUE:
            if (payload.length == 3) {
                //  red(payload.data[0]);
                //  green(payload.data[1]);
                //  blue(payload.data[2]);
            }
            break;
        case CMD_ON:
            if (payload.length == 1) {
                if (payload.data[0] == 0) {
                    //    red(255);
                } else if (payload.data[0] == 1) {
                    //    green(255);
                } else {
                    //   blue(255);
                }
            }
            break;
        case CMD_OFF:
            if (payload.length == 1) {
                if (payload.data[0] == 0) {
                    //     red(0);
                } else if (payload.data[0] == 1) {
                    //      green(0);
                } else {
                    //      blue(0);
                }
            } else {
                //   red(0);
                //   green(0);
                //   blue(0);
            }
            break;
    }
    return NULL;
}

void HomeNetDevice5Button::setLED(byte count) {

    switch (count) {
        case 0: //off
            setByte(IODIR, B11111111);
            setByte(GPIO, B00000000);
            break;
        case 1: //Button 1
            setByte(IODIR, B01111111);
            setByte(GPIO, B00000000);
            break;
        case 2: //Button 2
            setByte(IODIR, B01111111);
            setByte(GPIO, B10000000);
            break;
        case 3: //Button 3
            setByte(IODIR, B10111111);
            setByte(GPIO, B00000000);
            break;
        case 4: //Button 4
            setByte(IODIR, B10111111);
            setByte(GPIO, B01000000);
            break;
    }
    //delay(100);
}

byte HomeNetDevice5Button::getButton() {

    byte button = getByte(INTCAP);

    if (bitRead(lastButton, 5) != bitRead(button, 5)) {

        if (bitRead(button, 5) == 0) {
            buttonsEnabled = true;
            lastButton = button;
            return 6;
#ifdef DEBUG
            Serial.println("Buttons Enabled");
#endif
        } else {
#ifdef DEBUG
            Serial.println("Buttons Disabled");
#endif
            buttonsEnabled = false;
            setLED(0);
            lastButton = button;
            return 6;
        }
    }

   if (buttonsEnabled == true) {

        if ((bitRead(button, 0) == 1) && (bitRead(lastButton, 0) != bitRead(button, 0))) {
            setLED(1);
#ifdef DEBUG

#endif
            lastButton = button;
             return 1;
        }

        if ((bitRead(button, 1) == 1) && (bitRead(lastButton, 1) != bitRead(button, 1))) {
            setLED(2);
#ifdef DEBUG
            Serial.println("button 2");
#endif
            lastButton = button;
            return 2;
           
        }

        if ((bitRead(button, 2) == 1) && (bitRead(lastButton, 2) != bitRead(button, 2))) {
            setLED(3);
#ifdef DEBUG
            Serial.println("button 3");
#endif
            lastButton = button;
            return 3;
            
        }

        if ((bitRead(button, 3) == 1) && (bitRead(lastButton, 3) != bitRead(button, 3))) {
            setLED(4);
#ifdef DEBUG
            Serial.println("button 4");
#endif
            lastButton = button;
            return 4;
           
        }
        if ((bitRead(button, 4) == 1) && (bitRead(lastButton, 4) != bitRead(button, 4))) {
            setLED(0);
#ifdef DEBUG
            Serial.println("button 5");
#endif
            lastButton = button;
            return 5;
           
        }

    }

    lastButton = button;
    return 0;
}

void HomeNetDevice5Button::registerButtons(Button buttons[], uint8_t count){
    _buttonCount = count;
    _buttons = buttons;
}

