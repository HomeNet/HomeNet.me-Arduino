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

#ifndef HomeNetDeviceLCD_h
#define HomeNetDeviceLCD_h

#include <WProgram.h>
#include <stdint.h>

#include <../../../../libraries/Ports/PortsLCD.h>

//#include <HomeNet.h>
//#include <HomeNetConfig.h>
//#include <HomeNetDevices.h>

////////////////////////////LCD////////////////////////////

class HomeNetDeviceLCD : public HomeNetDevicePortI2C {
public:
    typedef struct {
        uint8_t row;
        uint16_t fromNode;
        uint8_t fromDevice;
        char message[17];
    } Display;

    HomeNetDeviceLCD(HomeNet& homeNet ) : HomeNetDevicePortI2C(homeNet) {}
    inline uint16_t getId(){ return 0x0009; }
    void init();
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t&, const HomeNetPayload& );
    void registerDisplay(Display[], uint8_t);

private:
    LiquidCrystalI2C * _lcd;
    Display * _display;
    uint8_t _displayCount;
};
#endif