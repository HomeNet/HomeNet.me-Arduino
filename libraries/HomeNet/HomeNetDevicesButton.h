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
 
#ifndef HomeNetDevicesButton_h
#define HomeNetDevicesButton_h

#include <WProgram.h>
#include <stdint.h>

#include <../../../../libraries/Ports/Ports.h>

#include <HomeNet.h>
#include <HomeNetConfig.h>
#include <HomeNetDevices.h>

//5 button Panel////////////////////////////////////////////////////////////////////////

class HomeNetDevice5Button : public HomeNetDeviceI2C {

public:

    enum {
      IODIR, IPOL, GPINTEN, DEFVAL, INTCON, IOCON,
      GPPU, INTF, INTCAP, GPIO, OLAT
    };

    typedef struct {
      const int8_t button;
      const int8_t type;
      const uint16_t toNode;
      const uint8_t toDevice;
      const uint8_t command;
      const HomeNetPayload payload;
    } Button;

    HomeNetDevice5Button(HomeNet& homeNet ):HomeNetDeviceI2C(0x20, homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
    HomeNetPartialPacket* interrupt(const uint8_t&, const HomeNetPayload&);
    void registerButtons(Button[], uint8_t);

protected:
    Button * _buttons;
    uint8_t _buttonCount;
    void setLED(byte count);
    byte lastButton;
    boolean buttonsEnabled;
    byte getButton();

};

#endif