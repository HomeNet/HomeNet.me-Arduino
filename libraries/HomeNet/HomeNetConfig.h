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

#ifndef HomeNetConfig_h
#define HomeNetConfig_h

#define SEND_RECEIVE 0
#define SEND_ONLY 1

#define ENABLE_SERIAL 1
#define ENABLE_RF12 1
//#define DEBUG 0

//remove success/fail out of the stack and directly clear packets
//#define REDUCE_STACK

#define SERIAL_SPEED 115200

#define DEVICE_UPDATE 1000

//schedule timer (millis)  = DEVICE_SCHEDULE * DEVICE_UPDATE;
#define DEVICE_SCHEDULE 1
  
#define PACKET_BUFFER 4
#define PACKET_TIMEOUT 500
#define PACKET_RETRY 1

//Max size of Packet
#define PACKET_SIZE 46
#define PAYLOAD_SIZE 36

#define TEMPERATURE_FORMAT  1 //0 = c, 1 = F

//0 sends it as a broadcast to all, 1 sends packet to base station only
#define RF12_NODE_OUT_OF_RANGE_SEND_TO 1 //not working yet
#define RF12_DEFAULT_FREQ RF12_915MHZ
#define RF12_DEFAULT_GROUP 33

#endif
