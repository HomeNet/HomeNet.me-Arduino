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

#ifndef HomeNet_h
#define HomeNet_h
 
#include <WProgram.h>
#include <stdint.h>
#include <util/crc16.h>

#include <../../../../libraries/Ports/Ports.h>
#include <../../../../libraries/RF12/RF12.h>

#include <HomeNetConfig.h>
  
void * operator new(size_t size);
void operator delete(void * ptr);

//setup interrupt support

//EMPTY_INTERRUPT(WDT_vect);

void onInterrupt();

#define CMD_ERROR           0x00
#define CMD_VERSION         0x01
#define CMD_BATTERYLEVEL    0x03
#define CMD_MEMORYFREE      0x04
#define CMD_PING            0x33
#define CMD_PONG            0x3E
#define CMD_ACK             0x11
#define CMD_SETNODEID       0x21
#define CMD_GETNODEID       0x21
#define CMD_SETNODEID       0x22
#define CMD_GETDEVICE       0x23
#define CMD_SETDEVICE       0x24

#define CMD_GETVALUE        0x90
#define CMD_SETVALUE        0x91

#define CMD_AUTOSENDSTART   0xB1
#define CMD_AUTOSENDSTOP    0xB2

#define CMD_ON              0xC0
#define CMD_OFF             0xC1
#define CMD_LEVEL           0xC2
#define CMD_CLEAR           0xC3

#define CMD_GETVALUE        0xD0
#define CMD_GETBYTE         0xD1
#define CMD_GETSTRING       0xD2
#define CMD_GETINT          0xD3
#define CMD_GETFLOAT        0xD4
#define CMD_GETLONG         0xD5
#define CMD_GETBINARY       0xD6
#define CMD_GETBOOLEAN      0xD7

#define CMD_SETVALUE        0xE0
#define CMD_SETBYTE         0xE1
#define CMD_SETSTRING       0xE2
#define CMD_SETINT          0xE3
#define CMD_SETFLOAT        0xE4
#define CMD_SETLONG         0xE5
#define CMD_SETBINARY       0xE6
#define CMD_SETBOOLEAN      0xE7

#define CMD_REPLYVALUE      0xF0
#define CMD_REPLYBYTE       0xF1
#define CMD_REPLYSTRING     0xF2
#define CMD_REPLYINT        0xF3
#define CMD_REPLYFLOAT      0xF4
#define CMD_REPLYLONG       0xF5
#define CMD_REPLYBINARY     0xF6
#define CMD_REPLYBOOLEAN    0xF7
#define CMD_REPLYERROR      0xFF

#define CMD_MULTIPACKET     0xA0


#define PACKET_LENGTH   0
#define PACKET_SETTINGS 1
#define PACKET_FROM     2
#define PACKET_TO       4
#define PACKET_TTL      4
#define PACKET_ID       6
#define PACKET_COMMAND  7
#define PACKET_PAYLOAD  8

//an enum might be better here
//enum Status { STATUS_CLEAR, STATUS_RECEVING, STATUS_RECEIVED, STATUS_WAITING, STATUS_READY, STATUS_SENDING, STATUS_SENT, STATUS_ACK, STATUS_SUCCESS, STATUS_FAILED }
#define STATUS_CLEAR    0
#define STATUS_RECEVING 1
#define STATUS_RECEIVED 2
#define STATUS_WAITING  3
#define STATUS_READY    4
#define STATUS_SENDING  5
#define STATUS_SENT     6
#define STATUS_ACK      7

#ifndef REDUCE_STACK
#define STATUS_SUCCESS  8
#define STATUS_FAILED  9
#else
#define STATUS_SUCCESS  0
#define STATUS_FAILED  0
#endif

#define OFFSET_PACKET 10
#define OFFSET_HEADER 8
#define OFFSET_FOOTER 2

#define PORT_SERIAL0  0
#define PORT_SERIAL  0
#define PORT_SERIAL1 1
#define PORT_SERIAL2 2
#define PORT_SERIAL3 3

#define PACKET_TCP       1
#define PACKET_UDP       2
#define PACKET_BROADCAST 3

#ifndef PACKET_BUFFER
#define PACKET_BUFFER 10
#endif

#ifndef PACKET_TIMEOUT
#define PACKET_TIMEOUT 100
#endif

#define COLLECT 0x20 // collect mode, i.e. pass incoming without sending acks

//payload type
class HomeNetPayload {
public:
    HomeNetPayload(const uint8_t);
    HomeNetPayload(const uint16_t);
    HomeNetPayload(const int);
    HomeNetPayload(const String&);
    HomeNetPayload(const uint8_t[], const uint8_t);
    HomeNetPayload(const char[]);
    HomeNetPayload(const float);
    HomeNetPayload(const float*, const uint8_t);
    HomeNetPayload(const long);
    HomeNetPayload(const bool);
    inline HomeNetPayload() { }
    inline uint8_t getByte(uint8_t place = 0) const { return data[place]; }
    inline int getInt() const { return (data[1] << 8) | data[0]; }
    inline int getLength() const { return length; }
    float getFloat() const;
    uint8_t length;
    uint8_t data[PAYLOAD_SIZE];
};

typedef struct {
  uint8_t fromDevice;
  uint8_t command;
  HomeNetPayload payload;
} HomeNetPartialPacket;

class HomeNetDevice; //pre declare this to solve link error

typedef struct {
  uint8_t delay; //seconds
  uint8_t frequency;    //seconds
  HomeNetDevice * device;
  const uint16_t toNode;
  const uint8_t toDevice;
  const uint8_t command;
  const HomeNetPayload payload;
} HomeNetSchedule;

typedef struct {
  HomeNetDevice * device;
  const uint16_t toNode;
  const uint8_t toDevice;
  const uint8_t command;
  const HomeNetPayload payload;
} HomeNetInterrupt;

HomeNetPartialPacket* partialPacket(const uint8_t&, const uint8_t&, const HomeNetPayload&);

class PayloadBuffer : public Print {

public:
  HomeNetPayload stor;
  inline PayloadBuffer() { reset(); }
  inline uint8_t* data() { return stor.data; }
  char* getString();
  inline HomeNetPayload payload() { return stor; }
  inline uint8_t length() { return stor.length; }
  inline void reset() { stor.length = 0; }
  void write(uint8_t ch);
};

class HomeNetPacket {

public:
    uint8_t toPort;
    uint8_t fromPort;
    uint8_t status;
    uint8_t data[PACKET_SIZE];

    inline HomeNetPacket() {}

    inline uint8_t  getToPort()    { return  toPort;}
    inline uint8_t  getFromPort()  { return  fromPort;}
    inline uint8_t  getStatus()    { return  status;}

    inline uint8_t  getLength()    { return  data[PACKET_LENGTH];}
    inline uint8_t  getSettings()  { return  data[PACKET_SETTINGS] & 0x0F; }
    inline uint8_t  getType    ()  { return  data[PACKET_SETTINGS] & 0x0F; }
    inline uint16_t getFromNode()  { return (data[PACKET_FROM] << 4) | (data[PACKET_FROM+1] >> 4); }
    inline uint8_t  getFromDevice(){ return  data[PACKET_FROM+1] & 0x0F;}
    inline uint16_t getToNode()    { return (data[PACKET_TO] << 4) | (data[PACKET_TO+1] >> 4); }
    inline uint8_t  getToDevice()  { return  data[PACKET_TO+1] & 0x0F;}
    inline uint8_t  getId()        { return  data[PACKET_ID];}
    inline uint8_t  getCommand()   { return  data[PACKET_COMMAND];}
    HomeNetPayload  getPayload();
           uint8_t  getPayloadAt(uint8_t);
    inline uint8_t  getPayloadLength()   { return  data[PACKET_LENGTH] - OFFSET_PACKET;}
    
           uint16_t getChecksum();
           
           
    inline void setToPort(uint8_t value)    { toPort = value;}
    inline void setFromPort(uint8_t value)  { fromPort = value;}
    inline void setStatus(uint8_t value)    { status = value;}
    
    inline void setLength(uint8_t value)    { data[PACKET_LENGTH]    = value;}
    inline void setSettings(uint8_t value)  { data[PACKET_SETTINGS] |= value & 0x0F; }
    inline void setType    (uint8_t value)  { data[PACKET_SETTINGS] |= value & 0x0F; }
           void setFromNode(uint16_t);
           void setFromDevice(uint8_t);
           void setToNode(uint16_t);
           void setToDevice(uint8_t);
    inline void setId(uint8_t value)        { data[PACKET_ID]       = value;}
    inline void setCommand(uint8_t value)   { data[PACKET_COMMAND]  = value;}

           void setPayload(const HomeNetPayload&);
    inline void setPayloadLength(uint8_t value) { data[PACKET_LENGTH] = value + OFFSET_PACKET;}
};

class HomeNet; //pre declare this to solve link error


class HomeNetPort {

public:
    inline HomeNetPort(HomeNet& homeNet){ _homeNet = &homeNet; }
    virtual void init(uint8_t) =0;
    virtual void send(HomeNetPacket*)=0;
    virtual void receive()= 0;
    inline boolean isSending() { return _sending; }
    inline boolean isReceiving() { return _receiving; }
    inline boolean getId() { return _id; }

protected:
    HomeNet * _homeNet;
    uint8_t _id;
    boolean _sending;// = false;
    boolean _receiving;// = false;
};

class HomeNetDevice {

public:
    inline HomeNetDevice(HomeNet& homeNet){ _homeNet = &homeNet; }

    inline void setLocation(uint8_t location) { _location = location; }

   // void setLocation(uint8_t location);

    inline uint8_t getLocation() { return _location; }

    virtual uint16_t getId() = 0;
    
    virtual inline void init() { }
    virtual inline void update(){ }
    virtual inline void updateOften(){ }

    virtual inline HomeNetPartialPacket* process  (const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload){ return process(command, payload); }
    virtual inline HomeNetPartialPacket* process  (const uint8_t& command, const HomeNetPayload& payload){ return process(command); }
    virtual inline HomeNetPartialPacket* process  (const uint8_t& command){ return NULL; }
    virtual inline HomeNetPartialPacket* interrupt(const uint8_t& command, const HomeNetPayload& payload){ return process(0, 0, command, payload); }
    virtual inline HomeNetPartialPacket* schedule (const uint8_t& command, const HomeNetPayload& payload){ return process(0, 0, command, payload); }

    

protected:

    uint8_t _location;
    HomeNet * _homeNet;
    
};

//Port//////////////////////////////////////////////////////////////////////////
class HomeNetDevicePort : public HomeNetDevice {
public:
    HomeNetDevicePort(HomeNet& homeNet ):HomeNetDevice(homeNet) {};
    inline virtual uint16_t getId() = 0;
    inline void init(){ _port = new Port(_location); }

protected:
    Port * _port;
};

//PortI2C///////////////////////////////////////////////////////////////////////
class HomeNetDevicePortI2C : public HomeNetDevice {
public:
    HomeNetDevicePortI2C(HomeNet& homeNet ):HomeNetDevice(homeNet) {};
    inline virtual uint16_t getId() = 0;
    inline void init(){_port = new PortI2C(_location); }

protected:
    PortI2C * _port;
};

//I2C///////////////////////////////////////////////////////////////////////
class HomeNetDeviceI2C : public HomeNetDevicePortI2C {
public:
    HomeNetDeviceI2C(byte i2cAddress, HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet), _i2cAddress(i2cAddress) {};
    HomeNetDeviceI2C(HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet), _i2cAddress(0x20) {};
    inline virtual uint16_t getId() = 0;
    void init();

protected:
    DeviceI2C * _i2c;
    void setByte(byte reg, byte value) const;
    byte getByte(byte reg) const;
    byte _i2cAddress;
};

///StatusLights/////////////////////////////////////////////////////////////////
class HomeNetDeviceStatusLights : public HomeNetDevicePort {
public:

    HomeNetDeviceStatusLights(HomeNet& homeNet):HomeNetDevicePort(homeNet) { }

    inline uint16_t getId() { return 0x0002; }
    void init();
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t&, const HomeNetPayload&);
    void update();
    void ledA(uint8_t on);
    void ledB(uint8_t on);

private:
    uint8_t _statusA;
    uint8_t _statusB;
};

////////////////////////////HomeNet/////////////////////////////////////////



class HomeNet{

public:
    HomeNet(uint16_t id);
    void init(HomeNetPort* [], uint8_t, HomeNetDevice* [],uint8_t);
    void receive();
    HomeNetPacket *getNewPacket();
    HomeNetPacket* clonePacket(HomeNetPacket*);
    boolean process();
    boolean processPacket(HomeNetPacket*);
    boolean processCommand(HomeNetPacket*);

    HomeNetPayload packetToPayload(HomeNetPacket*);

    HomeNetPacket* addTcpPacket(const uint8_t, const uint16_t, const uint8_t, const uint8_t, const HomeNetPayload& );
    HomeNetPacket* addUdpPacket(const uint8_t, const uint16_t, const uint8_t, const uint8_t, const HomeNetPayload& );
    HomeNetPacket* addBroadcastPacket(const uint8_t, const uint8_t, const HomeNetPayload);
    HomeNetPacket* addCrc(HomeNetPacket*);
    void debugPacket(HomeNetPacket*);
    int availableMemory();
    void debugPayload(const HomeNetPayload&);

    inline void registerStatusLights(HomeNetDeviceStatusLights& l) { statusLights = &l; }
    
    void registerSchedule(HomeNetSchedule[], uint8_t );
    void registerInterrupts(HomeNetInterrupt[], uint8_t ); 
    void deviceUpdate();
    void deviceSchedule();
    void deviceInterrupt(boolean = false);
    void loop();
    void sleep(uint8_t = 0);
    uint16_t getDeviceId(uint8_t);
    inline uint8_t getVersion(){ return 0x01; };

    uint16_t _nodeId;
    HomeNetDeviceStatusLights * statusLights;
    volatile boolean runInterupt;

private:
    HomeNetPort** _ports;
    uint8_t _portCount;
    HomeNetDevice** _devices;
    uint8_t _deviceCount;
    uint8_t _uniqueId;
    HomeNetPacket _packets[PACKET_BUFFER];
    HomeNetSchedule * _deviceSchedule;
    uint8_t _deviceScheduleCount;
    HomeNetInterrupt * _deviceInterrupts;
    uint8_t _deviceInterruptCount;
    uint8_t _scheduleCount;
    long _deviceTimer;
    uint8_t _getId();
};

class HomeNetPortSerial : public HomeNetPort{

public:
    HomeNetPortSerial(HomeNet& homeNet, uint8_t type = 0) : HomeNetPort(homeNet), _type(type) {};
    void init(uint8_t);
    void send(HomeNetPacket*);
    void receive();
    void buildPacket(uint8_t);

private:
    uint8_t _ptr;
    uint8_t _type;
    uint16_t _receivingChecksum;
    HomeNetPacket *_sendingPacket;
    HomeNetPacket *_receivingPacket;
    uint8_t _sendingRetryCount;
    long _receivingTimer;
    long _sendingTimer;
};

class HomeNetPortRF12 : public HomeNetPort{

const uint8_t _freq;
const uint8_t _group;
const uint8_t _mode; // 0 transmit and receive , 1 transmit only to save power

public:
    HomeNetPortRF12(HomeNet& homeNet, uint8_t mode = SEND_RECEIVE, uint8_t freq = RF12_DEFAULT_FREQ, uint8_t group = RF12_DEFAULT_GROUP) : HomeNetPort(homeNet), _mode(mode), _freq(freq), _group(group) { _waiting = false; };
    void init(uint8_t);
    void send(HomeNetPacket*);
    void receive();

private:
    boolean _sleeping;
    boolean _waiting;
};

typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
} RGB;

#endif