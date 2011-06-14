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
#include <WProgram.h>

//Arduino///////////////////////////////////////////////////////////////////////

HomeNetPartialPacket* HomeNetDeviceArduino::process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload) {
#ifdef DEBUG
    Serial.print("Arduino Processing: ");
    Serial.println(command, DEC);
#endif

    switch (command) {
        case CMD_ERROR:
            //addUdpPacket(0, 0, 0, CMD_PONG, getPayload(packet));
            break;
        case CMD_VERSION:
            return partialPacket(_location, CMD_REPLYBYTE, HomeNetPayload(_homeNet->getVersion()));
            break;
        case CMD_MEMORYFREE:
            return partialPacket(_location, CMD_REPLYINT, HomeNetPayload(_homeNet->availableMemory()));
            break;
        case CMD_PING:
            return partialPacket(_location, CMD_PONG, payload);
            break;
        case CMD_GETNODEID:
            return partialPacket(_location, CMD_REPLYINT, HomeNetPayload(_homeNet->_nodeId));
            break;
        case CMD_SETNODEID:
            //return partialPacket(_location, CMD_PONG, payload);
            break;
        case CMD_GETDEVICE: {
            uint16_t id = _homeNet->getDeviceId(payload.getByte(0));
            if(id > 0){
                return partialPacket(_location, CMD_REPLYINT, HomeNetPayload(id));
            } else {
                return partialPacket(_location, CMD_REPLYERROR, 0);
            }
            break; }
        case CMD_SETDEVICE:
            //return partialPacket(_location, CMD_PONG, payload);
            break;
    }
    return NULL;
}

//JeeNode///////////////////////////////////////////////////////////////////////

HomeNetPartialPacket* HomeNetDeviceJeeNode::process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload) {
#ifdef DEBUG
    Serial.print("JeeNode Processing: ");
    Serial.println(command, DEC);
#endif

    HomeNetPartialPacket* partial = HomeNetDeviceArduino::process(fromNode, fromDevice, command, payload);
    if(partial != NULL){
        return partial;
    }

    switch (command) {
        case CMD_BATTERYLEVEL:
            //return partialPacket(_location, CMD_PONG, payload);
            break;
    }
    return NULL;
}

//SimpleByte/////////////////////////////////////////////////////////////////////////

HomeNetPartialPacket* HomeNetDeviceStubByte::process(const uint8_t& command, const HomeNetPayload& payload)
{
    switch(command){

        case CMD_GETVALUE:
        case CMD_GETBYTE:
            return partialPacket(_location, CMD_REPLYBYTE, HomeNetPayload(getValue()));
            break;
    }
    return NULL;
}

//SimpleInt/////////////////////////////////////////////////////////////////////////

HomeNetPartialPacket* HomeNetDeviceStubInt::process(const uint8_t& command, const HomeNetPayload& payload)
{
    switch(command){

        case CMD_GETVALUE:
        case CMD_GETINT:
            return partialPacket(_location, CMD_REPLYINT, HomeNetPayload(getValue()));
            break;
    }
    return NULL;
}

//SimpleFloat/////////////////////////////////////////////////////////////////////////

HomeNetPartialPacket* HomeNetDeviceStubFloat::process(const uint8_t& command, const HomeNetPayload& payload)
{
    switch(command){

        case CMD_GETVALUE:
        case CMD_GETFLOAT:
            return partialPacket(_location, CMD_REPLYFLOAT, HomeNetPayload(getValue()));
            break;
    }
    return NULL;
}

//SimpleBoolean/////////////////////////////////////////////////////////////////
HomeNetPartialPacket* HomeNetDeviceStubBool::process(const uint8_t& command, const HomeNetPayload& payload)
{
    switch(command){

        case CMD_GETVALUE:
        case CMD_GETBOOLEAN:
            return partialPacket(_location, CMD_REPLYBOOLEAN, HomeNetPayload(getValue()));
            break;
    }
    return NULL;
}

//SimpleOnOff/////////////////////////////////////////////////////////////////
HomeNetPartialPacket* HomeNetDeviceStubOnOff::process(const uint8_t& command, const HomeNetPayload& payload)
{
    switch(command){
        case CMD_GETVALUE:
        case CMD_GETBOOLEAN:
            return partialPacket(_location, CMD_REPLYBOOLEAN, HomeNetPayload(_status));
            break;

         case CMD_ON:
             _status = true;
             on();
            break;

         case CMD_OFF:
             _status = false;
             off();
            break;
    }
    return NULL;
}










////////////////////////////TMP37////////////////////////////
float HomeNetDeviceTMP37::getValue() {
    _port->mode2(INPUT); // set pin to input
    float value = _port->anaRead();

    //Based on code from Modern Device: http://shop.moderndevice.com/products/temperature-sensor
    float celsius = (value * 3.3 / 1024.0) / .02; //celsius
    //float farenheit = (((value * 5.0 / 1024.0) / .02) * 9.0 / 5.0) + 32; //farenheit

    return celsius;
   // float farenheit = (((value * 3.3 / 1024.0) / .02) * 9.0 / 5.0) + 32; //farenheit
   // return farenheit;
}

//TMP421////////////////////////////////////////////////////////////////////////
void HomeNetDeviceTMP421::init() {
#ifdef DEBUG
    Serial.println("Init TMP421");
#endif
    HomeNetDevicePortI2C::init();
    _i2c = new DeviceI2C(*_port, 0x2A);
}

HomeNetPartialPacket* HomeNetDeviceTMP421::process(const uint8_t& command, const HomeNetPayload& payload) {
    switch (command) {

        case CMD_GETVALUE:
        case CMD_GETFLOAT:
            return partialPacket(_location, CMD_REPLYFLOAT, HomeNetPayload(readSensor()));
            break;
    }
    return NULL;
}

float HomeNetDeviceTMP421::readSensor() {

    int8_t tempC_hi = 0;
    uint8_t tempC_lo = 0;
    float tempC = 0;

    //  float value = _port->anaRead();
    //Based on code from Modern Device: http://shop.moderndevice.com/products/temperature-sensor

    _i2c->send();
    _i2c->write(0x00); // high byte is on register 0
    _i2c->receive();
    tempC_hi = _i2c->read(1);
    _i2c->stop();

    _i2c->send();
    _i2c->write(0x10); // low byte is on register 0x10
    _i2c->receive();
    tempC_lo = _i2c->read(1);
    _i2c->stop();

    tempC = (float) tempC_lo / 256; // make low byte into remainder
    tempC = tempC + tempC_hi;

//#if TEMPERATURE_FORMAT == 0
    // celsius
    return tempC;
//#else
    //farenheit
   // return ((tempC) * 9.0 / 5.0) + 32;
//#endif
}


//SHT21/////////////////////////////////////////////////////////////////////////
void HomeNetDeviceSHT21::init() {
#ifdef DEBUG
    Serial.println("Init SHT21");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
   HomeNetDevicePortI2C::init();
    _i2c = new DeviceI2C(*_port, 0x40);
}

HomeNetPartialPacket* HomeNetDeviceSHT21::process(const uint8_t& command, const HomeNetPayload& payload) {
    switch (command) {

        case CMD_GETVALUE:
        case CMD_GETFLOAT: {

            float data[2];
            data[0] = getTemp();
            data[1] = getHumidity();

            return partialPacket(_location, CMD_REPLYFLOAT, HomeNetPayload(data,2)); }
            break; 
    }
    return NULL;
}

float HomeNetDeviceSHT21::getTemp() {

    //Based on code from Modern Device: http://shop.moderndevice.com/products/humidity-and-temperature-sensor


    /*    eTempHoldCmd        = 0xE3,
    eRHumidityHoldCmd  = 0xE5,
    eTempNoHoldCmd      = 0xF3,
    eRHumidityNoHoldCmd = 0xF5,*/

    //uint8_t ack;
    
uint16_t result;
    //11100111

//uint8_t settings;
  //  _i2c->send();
   // _i2c->write(B11100110); //write to user register
   // delay(20);
  //  _i2c->write(B1010); //default system registers
   // delay(20);
   // _i2c->write(B11100111); // get user settings
  //  _i2c->receive();
  //  settings = _i2c->read(1);
  //  _i2c->stop();


    //get Temperature
    _i2c->send();
    _i2c->write(0xF3);
    _i2c->stop();
    delay(85);

    _i2c->send();
    _i2c->write(B10000001);
    _i2c->receive();
    result = ((_i2c->read(0)) << 8);
    result |= _i2c->read(1);
    _i2c->stop();

    result &= ~0x0003; //clear last 2bits

    return -46.85 + (175.72/65536.0) * (float) result;
}

float HomeNetDeviceSHT21::getHumidity() {

    uint16_t result;

    //get Humidity
    _i2c->send();
    _i2c->write(0xF5);
    _i2c->stop();

    delay(85);
    _i2c->send();
    _i2c->write(B10000001);
    _i2c->receive();
    result = ((_i2c->read(0)) << 8);
    result |= _i2c->read(1);
    _i2c->stop();

    result &= ~0x0003; //clear last 2bits

    return -6.0 + 125.0/65536.0 * (float)result;
}

//BMP085/////////////////////////////////////////////////////////////////////////
void HomeNetDeviceBMP085::init() {
#ifdef DEBUG
    Serial.println("Init BMP085");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
    HomeNetDevicePortI2C::init();
    _bmp085 = new BMP085(*_port,3);
    _bmp085->getCalibData();
}

HomeNetPartialPacket* HomeNetDeviceBMP085::process(const uint8_t& command, const HomeNetPayload& payload) {
    switch (command) {

        case CMD_GETVALUE:
        case CMD_GETLONG: {

            //getPressure();
            return partialPacket(_location, CMD_REPLYLONG, HomeNetPayload(getPressure()));
            break; }
    }
    return NULL;
}

long HomeNetDeviceBMP085::getPressure() {

    int16_t temp;
    long pres;

_bmp085->startMeas(BMP085::TEMP);
    delay(16);
    _bmp085->getResult(BMP085::TEMP);
//
    _bmp085->startMeas(BMP085::PRES);
delay(32);
    _bmp085->getResult(BMP085::PRES);
//
//   // struct {  } payload;
   _bmp085->calculate(temp, pres);
#ifdef DEBUG
    Serial.print("Pressure: ");
    Serial.println(pres, DEC);
#endif
   return pres;
}


//LDR///////////////////////////////////////////////////////////////////////////
int HomeNetDeviceLDR::getValue(){
   _port->mode2(INPUT);           // set pin to input
   _port->digiWrite2(HIGH);
   uint16_t value = map(_port->anaRead(),0,1023,100,0);
   _port->digiWrite2(LOW);
   return value;
}

//SWITCH////////////////////////////////////////////////////////////////////////

void HomeNetDeviceSwitch::init() {
#ifdef DEBUG
    Serial.println("Switch Initialized");
#endif
    HomeNetDevicePort::init();
    
    //attach interrupt
   // attachInterrupt(1, onInterrupt, CHANGE);

    _port->mode3(INPUT);
    _port->digiWrite3(HIGH);//turn on internal pullup
}

HomeNetPartialPacket* HomeNetDeviceSwitch::process(const uint8_t& command, const HomeNetPayload& payload)
{
    if(_port->digiRead3() == HIGH){
        _state = false;
    } else {
        _state = true;
    }

    switch(command){

        case CMD_GETVALUE:
        case CMD_GETBOOLEAN:
            //Serial.println(_port->digiRead(),DEC);
            return partialPacket(_location, CMD_REPLYBOOLEAN, HomeNetPayload(_state));
            break;
    }
    return NULL;
}
/*
void  HomeNetDeviceSwitch::updateOften(){
    if(HomeNetDeviceSwitch::deviceInterrupt > 0){
        HomeNetDeviceSwitch::deviceInterrupt = 0;
        _homeNet->deviceInterrupt();
     }
}
*/
//CONTACT SWITCH////////////////////////////////////////////////////////////////

void HomeNetDeviceContactSwitch::init() {
#ifdef DEBUG
    Serial.println("Contact Switch Initialized");
#endif
    HomeNetDevicePort::init();
    attachInterrupt(1, onInterrupt, CHANGE); //Setup interrupt
    //turn on internal pullup
    //_port->mode(INPUT);
    //_port->digiWrite(HIGH);
    //setup the interrupt the same
    _port->mode3(INPUT);
    _port->digiWrite3(HIGH);
}

HomeNetPartialPacket* HomeNetDeviceContactSwitch::process(const uint8_t& command, const HomeNetPayload& payload) {
#ifdef DEBUG
    Serial.println("Contact Switch processing");
#endif
    if(_port->digiRead3() == HIGH){
        _state = true;
    } else {
        _state = false;
    }

    switch(command){
        case CMD_GETVALUE:
        case CMD_GETBOOLEAN:
            //Serial.println(_port->digiRead(),DEC);
            return partialPacket(_location, CMD_REPLYBOOLEAN, HomeNetPayload(_state));
            break;
    }
    return NULL;
}

//LED///////////////////////////////////////////////////////////////////////////
void HomeNetDeviceLED::init() {
#ifdef DEBUG
    Serial.println("LED Initialized");
#endif
    HomeNetDevicePort::init();

    //fade light up and down on boot
    for(int i = 0; i<=255; i++){
        led(i);
        delay(5);
    }
    delay(10);
    for(int i = 255; i>=0; i--){
        led(i);
        delay(5);
    }
}

HomeNetPartialPacket* HomeNetDeviceLED::process(const uint8_t& command, const HomeNetPayload& payload)
{
#ifdef DEBUG
    Serial.println("LED Processing");
#endif

    switch(command){
        case CMD_GETVALUE:
        case CMD_GETBYTE:
            return partialPacket(_location, CMD_REPLYBYTE, HomeNetPayload(_level));
            break;

         case CMD_SETBYTE:
         case CMD_REPLYBYTE:
         case CMD_LEVEL:
            led(payload.data[0]);
            break;

        case CMD_REPLYBOOLEAN:
        case CMD_SETBOOLEAN:
            if(payload.data[0] > 0){
                led(255);
            } else {
                led(0);
            }

            break;
         case CMD_ON:      
             led(255);
            break;

         case CMD_OFF:
            led(0);  
            break;
    }
    return NULL;
}

void HomeNetDeviceLED::led(uint8_t on) {
#ifdef DEBUG
    Serial.println("Light A Called");
#endif
    _level = on;
    _port->mode(OUTPUT);
    _port->anaWrite(_level);
}

//MotionSensor////////////////////////////////////////////////////////////////////////

void HomeNetDeviceMotionSensor::init() {
#ifdef DEBUG
    Serial.println(_timeout, DEC);
    Serial.println(" Motion Sensor Initialized");
#endif

    HomeNetDevicePort::init();

    //Setup interrupt
    //turn on internal pullup
    //_port->mode(INPUT);
    //_port->digiWrite(HIGH);
    //setup the interrupt the same
    _port->mode3(INPUT);
    _port->digiWrite3(HIGH);
    _state = 0;
    _lastState = 0;
    _count = 0;
}

void HomeNetDeviceMotionSensor::update() {
    if (_count > 0) {
        _count--;
    }
    if (_count == 1) {
#ifdef DEBUG
        Serial.println("Motion Sensor timeout");
#endif
       //this is a hack to get the intterupt to trigger 

        _homeNet->deviceInterrupt(true);
    }
}

HomeNetPartialPacket* HomeNetDeviceMotionSensor::interrupt(const uint8_t& command, const HomeNetPayload& payload) {
#ifdef DEBUG
    Serial.println("Motion Sensor Triggered");
#endif
    if (_port->digiRead3() == HIGH) {
        _state = 255;
        _count = _timeout + 1;
    } else {
        _state = 0;
    }
#ifdef DEBUG
    Serial.print(_state, DEC);
    Serial.println(" Motion state");
#endif
    if(_state != _lastState){
        _lastState = _state;
        return process(command, payload);
    }
    return NULL;
}

HomeNetPartialPacket* HomeNetDeviceMotionSensor::process(const uint8_t& command, const HomeNetPayload& payload)
{
    #ifdef DEBUG
    Serial.println("Motion Sensor process");
#endif
    switch(command){

        case CMD_GETVALUE:
        case CMD_GETBYTE:
            //Serial.println(_port->digiRead(),DEC);
            return partialPacket(_location, CMD_REPLYBYTE, HomeNetPayload(_state));
            break;
    }
    Serial.println(command, DEC);
    return NULL;
}


//SMARTOUTLET///////////////////////////////////////////////////////////////////

void HomeNetDeviceSmartOutlet::init() {
#ifdef DEBUG
    Serial.println("Outlet Initialized");
#endif
    HomeNetDevicePort::init();
    
    outlet1(1);
    outlet2(1);
    delay(200);
    outlet1(0);
    outlet2(0);
}


HomeNetPartialPacket* HomeNetDeviceSmartOutlet::process(const uint8_t& command, const HomeNetPayload& payload) {

    switch (command) {

        case CMD_GETVALUE:
        case CMD_GETBYTE:
            if (payload.length == 0) {

            } else {

            }
            break;
        case CMD_SETBYTE:
        case CMD_SETVALUE:
            if (payload.length == 2) {
                outlet1(payload.data[0]);
                outlet2(payload.data[1]);
            }
            break;
        case CMD_ON:
            if (payload.length == 1) {
                if (payload.data[0] == 0) {
                    outlet1(1);
                } else {
                    outlet2(1);
                }
            }
            break;
        case CMD_OFF:
            if (payload.length == 1) {
                if (payload.data[0] == 0) {
                    outlet1(0);
                } else {
                    outlet2(0);
                }
            }
            break;
    }
    return NULL;
}
void HomeNetDeviceSmartOutlet::outlet1(uint8_t on)
{
     _state1 = on;
    _port->mode(OUTPUT);
    if (on > 0) {
        _port->digiWrite(1);
        return;
    }
    _port->digiWrite(0);
}
void HomeNetDeviceSmartOutlet::outlet2(uint8_t on)
{
    _state2 = on;
    _port->mode2(OUTPUT);
    if (on > 0) {
        _port->digiWrite2(!1);
        return;
    }
    _port->digiWrite2(!0);
}

//Servo Open Close/////////////////////////////////////////////////////////////////////////
    void HomeNetDeviceServoOpenClose::init(){
        #ifdef DEBUG
    Serial.println("ServoOpenClose Initialized");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
    HomeNetDevicePort::init();
    _port->digiWrite2(HIGH);
      _currentPosition = 0;
      _position = 0;
      _servo = new Servo();
      _servo->attach(_location + 3);
          #ifdef DEBUG
          Serial.print("Open: ");
    Serial.println(_open, DEC);

        Serial.print("Close: ");
    Serial.println(_close, DEC);
  #endif

    }
    HomeNetPartialPacket* HomeNetDeviceServoOpenClose::process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload){

      switch(command){

          case CMD_GETVALUE:
              return partialPacket(_location, CMD_REPLYBYTE, HomeNetPayload(_position));
              break;
          case CMD_REPLYBYTE:
          case CMD_SETVALUE:
              if(payload.data[0] >128){
                _position = _open;
              } else {
                _position = _close;
              }
              break;

           case CMD_ON:
                _position = _open;
              break;
           case CMD_OFF:
                _position = _close;
              break;
      }
      return NULL;
    }

  void HomeNetDeviceServoOpenClose::updateOften(){
    if(_position != _currentPosition){

      if(_position < _currentPosition){
        _currentPosition--;

      } else if(_position > _currentPosition) {
        _currentPosition++;
      }
      _servo->write(_currentPosition);
      delay(5);
    }
  }

//RGBLED//////////////////////////////////////////////////////////////////////

void HomeNetDeviceRGBLED::init() {
#ifdef DEBUG
    Serial.println("Init RGB LED");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
    HomeNetDevicePortI2C::init();
    _dp = new DimmerPlug(*_port, 0x40);

    _dp->setReg(DimmerPlug::MODE1, 0x00);     // normal
    _dp->setReg(DimmerPlug::GRPPWM, 255);    // duty cycle 25%
    _dp->setReg(DimmerPlug::LEDOUT0, 255);

    _red = 0;
    _green = 0;
    _blue = 0;
}

HomeNetPartialPacket* HomeNetDeviceRGBLED::process(const uint8_t& command, const HomeNetPayload& payload) {
    switch(command){

        case CMD_GETBYTE:
        case CMD_GETVALUE: {
            PayloadBuffer buffer;
            buffer.print(_red);
            buffer.print(_green);
            buffer.print(_blue);
            return partialPacket(_location, CMD_REPLYBYTE, buffer.payload());
            break; }
         case CMD_REPLYBYTE:
         case CMD_SETBYTE:
         case CMD_SETVALUE:
             if(payload.length == 3){
                red(payload.data[0]);
                green(payload.data[1]);
                blue(payload.data[2]);
             }
            break;
         case CMD_ON:
            if(payload.length == 1){
                if(payload.data[0] == 0){
                    red(255);
                } else if(payload.data[0] == 1) {
                    green(255);
                } else {
                    blue(255);
                }
            }
            break;
         case CMD_OFF:
            if(payload.length == 1){
                if(payload.data[0] == 0){
                    red(0);
                } else if(payload.data[0] == 1) {
                    green(0);
                } else {
                    blue(0);
                }
            } else {
                red(0);
                green(0);
                blue(0);
            }
            break;
    }
    return NULL;
}

void HomeNetDeviceRGBLED::red(uint8_t level)  {
    _red  = level;
    _dp->setReg(DimmerPlug::PWM0, 255-level);
}
void HomeNetDeviceRGBLED::green(uint8_t level){
    _green = level;
    _dp->setReg(DimmerPlug::PWM1, 255-level);
}
void HomeNetDeviceRGBLED::blue(uint8_t level) {
    _blue = level;
    _dp->setReg(DimmerPlug::PWM2, 255-level);
}



//MoodLight//////////////////////////////////////////////////////////////////////

void HomeNetDeviceMoodLight::init() {
#ifdef DEBUG
    Serial.println("Init MoodLight");
#endif
    HomeNetDeviceRGBLED::init();
    _hue = 0;
}

HomeNetPartialPacket* HomeNetDeviceMoodLight::process(const uint8_t& command, const HomeNetPayload& payload) {
   switch(command){

        case CMD_GETVALUE:
            return partialPacket(_location, CMD_REPLYINT, HomeNetPayload(_hue));
            break;
         case CMD_REPLYINT:
         case CMD_SETINT:    
             setHue(payload.getInt());
            break;
         case CMD_ON:
                red(255);
                green(255);
                blue(255);
            break;
         case CMD_OFF:
                red(0);
                green(0);
                blue(0);
            break;
    }
    return NULL;
}

RGB HomeNetDeviceMoodLight::HSV2RGB(float h, float s, float v) {

    int h_i = ((int)(h/60)) % 6;

    float f = (h/60) - (int)(h/60);

    float r,g,b;

    float p = v * (1.0 - s);
    float q = v * (1.0 - f*s);
    float t = (1.0 - (1.0 - f)*s);

    switch(h_i) {
    case 0:  r = v; g = t; b = p; break;
    case 1:  r = q; g = v; b = p; break;
    case 2:  r = p; g = v; b = t; break;
    case 3:  r = p; g = q; b = v; break;
    case 4:  r = t; g = p; b = v; break;
    case 5:  r = v; g = p; b = q; break;
    }

    RGB color;

    color.r = 255 - (byte)((255 - 0)*r);
    color.g = 160 - (byte)((160 - 0)*g);
    color.b = 160 - (byte)((160 - 0)*b);

    return color;
}

void HomeNetDeviceMoodLight::setHue(int hue)  {
    //int offset = 200;

    //hue += offset;
    if(hue > 360){
      hue -= 360;
    } else if ( hue < 0){
      hue += 360;
    }
    _hue = hue;
    RGB color = HSV2RGB(_hue,1,.8);
    red(color.r); 
    green(color.g);
    blue(color.b);
  }