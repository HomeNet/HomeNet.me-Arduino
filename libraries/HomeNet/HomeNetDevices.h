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
 
#ifndef HomeNetDevices_h
#define HomeNetDevices_h

#include <WProgram.h>
#include <stdint.h>

#include <../../../../libraries/Servo/Servo.h> 
#include <../../../../libraries/Ports/Ports.h>
#include <../../../../libraries/Ports/PortsBMP085.h>

#include <HomeNet.h>
#include <HomeNetConfig.h> 

//Placeholder///////////////////////////////////////////////////////////////////////
class HomeNetDevicePlaceholder : public HomeNetDevice {

public:
    HomeNetDevicePlaceholder(HomeNet& homeNet ):HomeNetDevice(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    inline HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t&, const HomeNetPayload&) { return NULL;}
};

//Nodes//

//Arduino///////////////////////////////////////////////////////////////////////
class HomeNetDeviceArduino : public HomeNetDevice {

public:
    HomeNetDeviceArduino(HomeNet& homeNet ):HomeNetDevice(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t&, const HomeNetPayload&);
};
//JeeNode///////////////////////////////////////////////////////////////////////
class HomeNetDeviceJeeNode : public HomeNetDeviceArduino {

public:
    HomeNetDeviceJeeNode(HomeNet& homeNet ):HomeNetDeviceArduino(homeNet) {}
    inline uint16_t getId(){ return 0x0001; }
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t&, const HomeNetPayload&);
};



//Stubs//


//SimpleByte////////////////////////////////////////////////////////////////////
class HomeNetDeviceStubByte : public HomeNetDevicePort {
public:
    HomeNetDeviceStubByte(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

private:
    virtual uint8_t getValue();
};

//SimpleInt/////////////////////////////////////////////////////////////////////
class HomeNetDeviceStubInt : public HomeNetDevicePort {
public:
    HomeNetDeviceStubInt(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

private:
    virtual int getValue();
};

//SimpleFloat///////////////////////////////////////////////////////////////////
class HomeNetDeviceStubFloat : public HomeNetDevicePort {
public:
    HomeNetDeviceStubFloat(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

private:
    virtual float getValue();
};

//SimpleBoolean/////////////////////////////////////////////////////////////////
class HomeNetDeviceStubBool : public HomeNetDevicePort {
public:
    HomeNetDeviceStubBool(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {}
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

protected:
    virtual bool getValue();
};

//SimpleOnOff///////////////////////////////////////////////////////////////////
class HomeNetDeviceStubOnOff : public HomeNetDevicePort {
public:
    HomeNetDeviceStubOnOff(HomeNet& homeNet ):HomeNetDevicePort(homeNet) { _status = false; }
    inline uint16_t getId(){ return 0x0000; }
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

protected:
    bool _status;
    virtual void on();
    virtual void off();
};








//Sensors//

//TMP37/////////////////////////////////////////////////////////////////////////
class HomeNetDeviceTMP37 : public HomeNetDeviceStubFloat {
public:
    HomeNetDeviceTMP37(HomeNet& homeNet ):HomeNetDeviceStubFloat(homeNet) {}
    inline uint16_t getId(){ return 0x0003; }

protected:
    float getValue();
};

//TMP421////////////////////////////////////////////////////////////////////////

class HomeNetDeviceTMP421 : public HomeNetDevicePortI2C {
public:
    HomeNetDeviceTMP421(HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

private:
    DeviceI2C * _i2c;
    float readSensor();
};

//SHT21////////////////////////////////////////////////////////////////////////
class HomeNetDeviceSHT21 : public HomeNetDevicePortI2C {
public:
    HomeNetDeviceSHT21(HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
 
private:
    DeviceI2C * _i2c;
    float getTemp();
    float getHumidity();
};

//BMP085////////////////////////////////////////////////////////////////////////
class HomeNetDeviceBMP085 : public HomeNetDevicePortI2C {
public:
    HomeNetDeviceBMP085(HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

private:
    BMP085 * _bmp085;
    long getPressure();
};

//LDR///////////////////////////////////////////////////////////////////////////
class HomeNetDeviceLDR : public HomeNetDeviceStubInt {
public:
    HomeNetDeviceLDR(HomeNet& homeNet ):HomeNetDeviceStubInt(homeNet) { };
    inline uint16_t getId(){ return 0x0005; };
    
protected:
    int getValue();
};

//Switch////////////////////////////////////////////////////////////////////////
class HomeNetDeviceSwitch : public HomeNetDevicePort {
private:
    

public:
    HomeNetDeviceSwitch(HomeNet& homeNet ):HomeNetDevicePort(homeNet) { };
    inline uint16_t getId(){ return 0x0006; };
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
    //void updateOften();


protected:
    uint8_t _state;
};

//ContactSwitch/////////////////////////////////////////////////////////////////

class HomeNetDeviceContactSwitch : public HomeNetDeviceSwitch {
public:
    HomeNetDeviceContactSwitch(HomeNet& homeNet ):HomeNetDeviceSwitch(homeNet) {};
    inline uint16_t getId(){ return 0x0007; };
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
};



//MotionSensor//////////////////////////////////////////////////////////////////

class HomeNetDeviceMotionSensor : public HomeNetDevicePort {
public:
    HomeNetDeviceMotionSensor(HomeNet& homeNet, uint8_t timeout = 60, uint8_t trigger = RISING  ):HomeNetDevicePort(homeNet), _timeout(timeout) {
        attachInterrupt(1, onInterrupt, trigger);
    };
    inline uint16_t getId(){ return 0x0010; };
    void init();
    HomeNetPartialPacket* interrupt(const uint8_t&, const HomeNetPayload&);
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
    void update();

private:
    uint8_t _state;
    uint8_t _lastState;
    uint8_t _timeout;
    uint8_t _count; 
};




//outputs//

//LED///////////////////////////////////////////////////////////////////////////

class HomeNetDeviceLED : public HomeNetDevicePort {

public:
    HomeNetDeviceLED(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {}
    inline uint16_t getId(){ return 0x0008; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
    void led(uint8_t on);

private:
   uint8_t _level;
};

//SmartOutlet///////////////////////////////////////////////////////////////////

class HomeNetDeviceSmartOutlet : public HomeNetDevicePort {
public:
    HomeNetDeviceSmartOutlet(HomeNet& homeNet ):HomeNetDevicePort(homeNet) {};
    inline uint16_t getId(){ return 0; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
    void outlet1(uint8_t on);
    void outlet2(uint8_t on);

private:
    boolean _state1;
    boolean _state2;
};

//ServoOpenClose///////////////////////////////////////////////////////////////////

class HomeNetDeviceServoOpenClose : public HomeNetDevicePort {

public:
    
    HomeNetDeviceServoOpenClose(HomeNet& homeNet ): HomeNetDevicePort(homeNet), _open(0), _close(180) { }
    HomeNetDeviceServoOpenClose(uint8_t open, uint8_t close, HomeNet& homeNet ):HomeNetDevicePort(homeNet) { _open = open; _close = close; }
    
    inline uint16_t getId(){ return 19; }
    void init();
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload);

    void updateOften();


private:
   Servo * _servo;
   uint8_t _currentPosition;
   uint8_t _position;
   uint8_t _open;
   uint8_t _close;
};

//RGBLED////////////////////////////////////////////////////////////////////////

class HomeNetDeviceRGBLED : public HomeNetDevicePortI2C {
public:
    HomeNetDeviceRGBLED(HomeNet& homeNet ):HomeNetDevicePortI2C(homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);

protected:
    DimmerPlug * _dp;
    
    uint8_t _red;
    uint8_t _green;
    uint8_t _blue;

    void red(uint8_t);
    void green(uint8_t);
    void blue(uint8_t);
};

//MoodLamp////////////////////////////////////////////////////////////////////////

class HomeNetDeviceMoodLight : public HomeNetDeviceRGBLED {
public:

    HomeNetDeviceMoodLight(HomeNet& homeNet ):HomeNetDeviceRGBLED(homeNet) {}
    inline uint16_t getId(){ return 0x0004; }
    void init();
    HomeNetPartialPacket* process(const uint8_t&, const HomeNetPayload&);
protected:
    int _hue;
    RGB HSV2RGB(float h, float s, float v);
    void setHue(int hue);
};














#endif