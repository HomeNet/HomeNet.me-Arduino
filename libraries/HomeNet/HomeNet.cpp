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

#include <HomeNetConfig.h>
#include "HomeNet.h"
#include <WProgram.h>
#include <avr/sleep.h>
#include <util/atomic.h>
#include <avr/wdt.h>
 
void * operator new(size_t size)
{
  return malloc(size);
}

void operator delete(void * ptr)
{
  free(ptr);
}

//ISR(WDT_vect) { Sleepy::watchdogEvent(); }


static volatile uint8_t HomeNetInterruptCount = 0;
//static volatile byte watchdogCounter;
void onInterrupt(){ 
    ++HomeNetInterruptCount;
     Sleepy::watchdogInterrupts(-1);
}

//HomeNetPayload////////////////////////////////////////////////////////////////

HomeNetPayload::HomeNetPayload(const uint8_t value) {
    length = 1;
    data[0] = value & 0xFF;
}

HomeNetPayload::HomeNetPayload(const uint8_t value[], const uint8_t size) {
    for (uint8_t i = 0; i < size; i++) {
        data[i] = value[i];
    }
    length = size;
}

HomeNetPayload::HomeNetPayload(const uint16_t value) {
    length = 2;
    data[1] = (value >> 8) &0xFF;
    data[0] = value & 0xFF;
}

HomeNetPayload::HomeNetPayload(const int value) {
    length = 2;
    data[1] = (value >> 8) &0xFF;
    data[0] = value & 0xFF;
}

HomeNetPayload::HomeNetPayload(const String & value) {
    PayloadBuffer buffer;
    buffer.print(value);
    length = buffer.length();
    for (uint8_t i = 0; i < length; i++) {
        data[i] = buffer.stor.data[i];
    }
}

HomeNetPayload::HomeNetPayload(const char value[]) {
    uint8_t i = 0;
    while (value[i] != NULL) {
        data[i] = value[i];
        i++;
    }
    length = i;
}

HomeNetPayload::HomeNetPayload(const float value) {
    union {
        float in;
        long out;
    } number;

    number.in = value;

    length = 4;

    data[3] = (number.out >> 24) &0xFF;
    data[2] = (number.out >> 16) &0xFF;
    data[1] = (number.out >> 8) &0xFF;
    data[0] = number.out & 0xFF;
}

HomeNetPayload::HomeNetPayload(const float * value, const uint8_t size) {
    union {
        float in;
        long out;
    } number;
    
    length = 4*size;
    
    uint8_t offset = 0;

    for(int i = 0; i < size; i++){
        number.in = value[i];
        offset = i*4;

        data[offset+3] = (number.out >> 24) &0xFF;
        data[offset+2] = (number.out >> 16) &0xFF;
        data[offset+1] = (number.out >> 8) &0xFF;
        data[offset+0] = number.out & 0xFF;
    }
}

HomeNetPayload::HomeNetPayload(const long value) {
    length = 4;
    data[3] = (value >> 24) &0xFF;
    data[2] = (value >> 16) &0xFF;
    data[1] = (value >> 8) &0xFF;
    data[0] = value & 0xFF;
}

HomeNetPayload::HomeNetPayload(const bool value) {
    length = 1;
    if(value == true){
        data[0] = 0xFF;
    } else {
        data[0] = 0x00;
    }
}


float HomeNetPayload::getFloat() const {
    union {
        float out;
        long in;
    } number;

    number.in = ((long)data[0] << 24) | ((long)data[1] << 16) | ((long)data[2] << 8) | data[3];
    return number.out;
}

//HomeNetPartialPacket//////////////////////////////////////////////////////////

HomeNetPartialPacket* partialPacket(const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload) {
    HomeNetPartialPacket* partial = new HomeNetPartialPacket;
    partial->fromDevice = fromDevice;
    partial->command = command;
    partial->payload = payload;
    return partial;
}

void HomeNet::debugPayload(const HomeNetPayload& payload) {

    Serial.println("==Debug Payload==");
    Serial.print("Length: ");
    Serial.println(payload.length, DEC);

    Serial.print("Payload: ");
    for (uint8_t i = 0; i < payload.length; i++) {
        Serial.print(payload.data[i], DEC);
        Serial.print(",");
    }
    Serial.println("");
}


//Payload Buffer////////////////////////////////////////////////////////////////

void PayloadBuffer::write(uint8_t ch)
{
  if (stor.length < sizeof stor.data){
    stor.data[stor.length++] = ch;
  }
}

char* PayloadBuffer::getString() {
    //char out[stor.length+1];
    char out[16];
    for(uint8_t i = 0; i<stor.length; i++){
        out[i] = (char)stor.data[i];
        Serial.print((char)stor.data[i]);
    }
    out[stor.length+1] = NULL;
    return out;
}

//HomeNetPacket/////////////////////////////////////////////////////////////////

void HomeNetPacket::setFromNode(uint16_t value) {
    data[PACKET_FROM] = (value >> 4) & 0xFF;
    data[PACKET_FROM + 1] &= 0x0F; //clear top 4 bits
    data[PACKET_FROM + 1] |= (value << 4) & 0xF0;
}

void HomeNetPacket::setToNode(uint16_t value) {
    data[PACKET_TO] = (value >> 4) & 0xFF;
    data[PACKET_TO + 1] &= 0x0F; //clear top 4 bits
    data[PACKET_TO + 1] |= (value << 4) & 0xF0;
}

void HomeNetPacket::setFromDevice(uint8_t value) {
    data[PACKET_FROM + 1] &= 0xF0;
    data[PACKET_FROM + 1] |= value & 0x0F;
}

void HomeNetPacket::setToDevice(uint8_t value) {
    data[PACKET_TO + 1] &= 0xF0;
    data[PACKET_TO + 1] |= value & 0x0F;
}

uint8_t HomeNetPacket::getPayloadAt(uint8_t place) {
    if ((getLength() - OFFSET_FOOTER) > (OFFSET_HEADER + place)) {
        return data[OFFSET_HEADER + place];
    }
    return NULL;
}

uint16_t HomeNetPacket::getChecksum() {
    return (data[getLength() - OFFSET_FOOTER] << 8) | data[getLength() - 1];
}

HomeNetPayload HomeNetPacket::getPayload()
{

  HomeNetPayload payload;
  payload.length = getPayloadLength();
  for(uint8_t i=0; i < payload.length; i++){
      payload.data[i] = data[i+OFFSET_HEADER];
  }
  return payload;
}

void HomeNetPacket::setPayload(const HomeNetPayload& payload) {
    setPayloadLength(payload.length);
    for (uint8_t i = 0; i < payload.length; i++) {
        data[i + OFFSET_HEADER] = payload.data[i];
    }
}

//HomeNetDevices////////////////////////////////////////////////////////////////
void HomeNetDeviceI2C::init() {
#ifdef DEBUG
    Serial.println("Init Device  I2C");
#endif
    HomeNetDevicePortI2C::init();
    _i2c = new DeviceI2C(*_port, 0x20);
}

void HomeNetDeviceI2C::setByte(byte reg, byte value) const {

    #ifdef DEBUG
    Serial.print("Set Byte: ");
    Serial.print(reg, DEC);
    Serial.print(" ");
    Serial.println(value, BIN);
    #endif


    _i2c->send();
    _i2c->write(reg);
    _i2c->write(value);
    _i2c->stop();
}

byte HomeNetDeviceI2C::getByte(byte reg) const {

    #ifdef DEBUG
    Serial.print("Read Byte: ");
    Serial.println(reg, DEC);
    #endif

    _i2c->send();
    _i2c->write(reg);
    _i2c->receive();
    byte result = _i2c->read(1);
    _i2c->stop();
    return result;
}


//StatusLights//////////////////////////////////////////////////////////////////

void HomeNetDeviceStatusLights::init() {

    HomeNetDevicePort::init();

#ifdef DEBUG
    Serial.println("Status Lights Initialized");
    Serial.print("Location: ");
    Serial.println(_location, DEC);
#endif
    pinMode(9, OUTPUT);
    digitalWrite(9,HIGH);


    ledA(1);
    ledB(1);
    delay(200);
    ledA(0);
    ledB(0);
}

HomeNetPartialPacket* HomeNetDeviceStatusLights::process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload)
{
#ifdef DEBUG
    Serial.println("Lights Processing");
#endif

    switch(command){
        case CMD_GETVALUE:
            if(payload.length == 0){

            } else {

            }
            break;
         case CMD_SETBYTE:
         case CMD_SETVALUE:
             if(payload.length == 2){
                ledA(payload.data[0]);
                ledB(payload.data[1]);
             }
            break;
         case CMD_ON:
            if(payload.length == 1){
                if(payload.data[0] == 0){
                    ledA(1);
                } else {
                    ledB(1);
                }
            }
            break;
         case CMD_OFF:
            if(payload.length == 1){
                if(payload.data[0] == 0){
                    ledA(0);
                } else {
                    ledB(0);
                }
            }
            break;
    }
    return NULL;
}

void HomeNetDeviceStatusLights::update() {
    //use update to turn off leds after x sec
    if (_statusA > 1) {
        _statusA--;
        if (_statusA == 1) {
            ledA(0);
            _statusA = 0;
        }
    }
    if (_statusB > 1) {
        _statusB--;
        if (_statusB == 1) {
            ledB(0);
            _statusB = 0;
        }
    }

}

void HomeNetDeviceStatusLights::ledA(uint8_t on) {

    _statusA = on;

    _port->mode(OUTPUT);
    if (on > 0) {
        _port->digiWrite(1);
        digitalWrite(9, LOW);
        return;
    }
    _port->digiWrite(0);
    digitalWrite(9,HIGH);

#ifdef DEBUG
    Serial.println("Light A Called");
#endif

}

void HomeNetDeviceStatusLights::ledB(uint8_t on) {
#ifdef DEBUG
    Serial.println("Light B Called");
#endif
    _statusB = on;
    
    _port->mode2(OUTPUT);

    if (on > 0) {
        _port->digiWrite2(!1);
        digitalWrite(9,LOW);
        return;
    }
    _port->digiWrite2(!0);
    digitalWrite(9,HIGH);
}

//HomeNet///////////////////////////////////////////////////////////////////////

HomeNet::HomeNet(uint16_t id) {
    _nodeId = id;
    _uniqueId = 0;
    _deviceScheduleCount = 0;
    _deviceInterruptCount = 0;
    statusLights = NULL;
    runInterupt = false;
}

void HomeNet::init(HomeNetPort* ports[], uint8_t portCount, HomeNetDevice* devices[], uint8_t deviceCount)//HomeNetPort** ports,
{
#ifdef DEBUG
    Serial.begin(SERIAL_SPEED);
    Serial.println("DEBUGGING MODE");
#endif

    _ports = ports;
    _portCount = portCount;
    _devices = devices;
    _deviceCount = deviceCount;
    //setup ports
    for (uint8_t i = 0; i < _portCount; i++) {
        _ports[i]->init(i);
    }
    //setup devices
    for (uint8_t i = 0; i < _deviceCount; i++) {
        _devices[i]->setLocation(i);
        _devices[i]->init();
    }
}

void HomeNet::receive() {
    for (uint8_t i = 0; i < _portCount; i++) {
        _ports[i]->receive();
        //_ports++;
    }
}

HomeNetPacket *HomeNet::getNewPacket() {
    for (uint8_t i = 0; i < PACKET_BUFFER; i++) {
        if (_packets[i].status == STATUS_CLEAR) {
            _packets[i].fromPort = 255;
            _packets[i].toPort = 255;
            return &_packets[i];
        }
    }
#ifdef DEBUG
  Serial.println("all full");
#endif
  //if stack is full bump data
  byte highest = 2, id;
  for(uint8_t i=0; i < PACKET_BUFFER; i++) {
    if(_packets[i].status >= highest ){
      id = i;
      highest = _packets[i].status;
    }
  }
    _packets[id].fromPort = 255;
    _packets[id].toPort = 255;
  return &_packets[id];
}

HomeNetPacket* HomeNet::clonePacket(HomeNetPacket* packet)
{
  HomeNetPacket* packet2 = getNewPacket();
  memcpy(packet2,packet, sizeof(*packet)) ;
  
  return packet2;
}

boolean HomeNet::process()
{
  boolean process = false; //allows this to be looped until the whole stack is clear
 //_sending = false; //only send one pack per cycle
  for(uint8_t i=0;i<PACKET_BUFFER;i++){
    if(processPacket(&_packets[i]) == true){
      process = true;
    }
  }

  /*
   * PayloadBuffer* buffer = new PayloadBuffer;
   PartialPacket* partial;
        partial = _devices[i]->schedule();
        if(partial != NULL){
            //check to make sure there is space in the packet
            ///*if((partial->payload->length + buffer.length())> 56){
                if(packetType == 2){
                    addUdpPacket(0,toNode,toDevice,CMD_MULTIPACKET,buffer.payload());
                    buffer.reset();
                }
            }//*//*
            //buffer.print("TEST",BYTE);
            buffer->print(partial->payload->length + 3,BYTE);
            buffer->print(partial->fromDevice,BYTE);
            buffer->print(partial->command,BYTE);
            for(uint8_t j=0; j< partial->payload->length; j++){
                buffer->print(partial->payload->data[j],BYTE);
            }
            //delete partial->payload;
            delete partial;
      if(packetType == 2){
        addUdpPacket(0,toNode,toDevice,CMD_MULTIPACKET,buffer->payload());
    }
    delete buffer;

   */
  return process;
}

boolean HomeNet::processPacket(HomeNetPacket *packet)
{
  switch (packet->status){
    case STATUS_CLEAR:
      #ifdef DEBUG
      //Serial.println("CLEAR");
      #endif
	return false;
      break;
    case STATUS_RECEVING:
      #ifdef DEBUG
      Serial.println("RECEVING");
      #endif
      break;
    case STATUS_RECEIVED:
      #ifdef DEBUG
      debugPacket(packet);
      #endif 
      
      if(packet->getToNode() == _nodeId){
        processCommand(packet);
        packet->status = STATUS_CLEAR;
      } else {
        packet->status = STATUS_READY;
      }
      #ifdef DEBUG
      Serial.println("RECEIVED");
      #endif
      break;
    case STATUS_WAITING:
      #ifdef DEBUG
      Serial.println("WAITING");
      #endif
      return false;
      break;
    case STATUS_READY:

      if(packet->getToNode() == _nodeId){ //check for packets to self and process them
        processCommand(packet);
        packet->setStatus(STATUS_CLEAR);
      }

       //set toPort address. a unique packet is required for each port
       //system built the packet and it needs to go to all the ports
       if(packet->fromPort == packet->toPort){ //the only times these are equal is when they both =
          packet->toPort = 0;
              for(uint8_t i=1; i<_portCount;i++){
                  HomeNetPacket *p = clonePacket(packet);

                    p->toPort = i;
                }
          //packet came from a port and needs to passed to the other ports
          } else if (packet->toPort == 255){ //has a fromPort but no toPort
              for(uint8_t i=0; i<_portCount;i++){
                  if(packet->fromPort != i){
                      if(packet->toPort == 255){
                          packet->toPort = i;
                      } else {
                          HomeNetPacket *p = clonePacket(packet);
                           p->toPort = i;
                      }
                }
              }
          }


        if(_ports[packet->toPort]->isSending() == false){
           packet->setStatus(STATUS_SENDING);
        } // only mark packet per type to send per loop. sending serial is blocking.



      //if reset pointer and set to send
      #ifdef DEBUG
      Serial.println("READY");
      #endif
      break;
    case STATUS_SENDING:
       //because of the above, only one packet will be changed to sending
        _ports[packet->toPort]->send(packet);
          
      #ifdef DEBUG
      Serial.println("SENDING");
      #endif
      break;
    case STATUS_SENT:
      //if is a ack packet 
      if(packet->getType() == 1){
        packet->setStatus(STATUS_ACK);;
        //start packets ack timer
      } else {
        //packet->status = STATUS_SUCCESS;
      }
      #ifdef DEBUG
      //_sending = false; //allow next packet to send
      packet->setStatus(STATUS_SUCCESS);
      Serial.println("SENT");
      #endif
      break;
    case STATUS_ACK:
       //if execeded timer resen
       //count resends if exceds max kill packet and add error
       #ifdef DEBUG
       Serial.println("ACK");
       #endif
      break;
#ifndef REDUCE_STACK
    case STATUS_SUCCESS:
      //option to post status to screen
      packet->setStatus(STATUS_CLEAR);
      #ifdef DEBUG
      Serial.println("SUCCESS");
      #endif
      break;
    case STATUS_FAILED:
      //option to post status to screen
      packet->setStatus(STATUS_CLEAR);
      #ifdef DEBUG
      Serial.println("FAILED");
      #endif
      break;
#endif
  }
return true;
}

boolean HomeNet::processCommand(HomeNetPacket *packet) {
    uint8_t device = packet->getToDevice();

    HomeNetPartialPacket* partial = _devices[device]->process(packet->getFromNode(), packet->getFromDevice(), packet->getCommand(), packet->getPayload());
    if (partial != NULL) {
        // if(packetType == 2){
        addUdpPacket(partial->fromDevice, packet->getFromNode(), packet->getFromDevice(), partial->command, partial->payload);
        delete partial;
    }

    return true;
}

HomeNetPayload HomeNet::packetToPayload(HomeNetPacket *packet)
{
  uint8_t length = packet->getLength();
  HomeNetPayload payload;

  for(uint8_t i=0; i < length; i++){
    payload.data[i] = packet->data[i];
  }
  return payload;
}

HomeNetPacket* HomeNet::addTcpPacket(const uint8_t fromDevice, const uint16_t toNode, const uint8_t toDevice, const uint8_t command, const HomeNetPayload& payload)
{
  HomeNetPacket *packet = getNewPacket();
  packet->setStatus(STATUS_READY);
  packet->setLength(payload.length + OFFSET_PACKET); //header 8 crc 2
  packet->setType(2);//1 //TCP packet not finished yet. tcp will send as udp for now
  packet->setFromNode(_nodeId);
  packet->setFromDevice(fromDevice);
  packet->setToNode(toNode);
  packet->setToDevice(toDevice);
  packet->setId(_getId());
  packet->setCommand(command);
  packet->setPayload(payload);

  return addCrc(packet);
}
 
HomeNetPacket* HomeNet::addUdpPacket(const uint8_t fromDevice, const uint16_t toNode, const uint8_t toDevice, const uint8_t command, const HomeNetPayload& payload)
{
  #ifdef DEBUG
    Serial.print("Packet Size: ");
    Serial.println(payload.length,DEC);
#endif

  HomeNetPacket *packet = getNewPacket();
  packet->setStatus(STATUS_READY);
  packet->setLength(payload.length + OFFSET_PACKET); //header 8 crc 2
  packet->setType(2);
  packet->setFromNode(_nodeId);
  packet->setFromDevice(fromDevice);
  packet->setToNode(toNode);
  packet->setToDevice(toDevice);
  packet->setId(_getId());
  packet->setCommand(command);
  packet->setPayload(payload);

  return addCrc(packet);
}
  
HomeNetPacket* HomeNet::addBroadcastPacket(const uint8_t fromDevice, const uint8_t ttl, const HomeNetPayload payload)
{
  HomeNetPacket *packet = getNewPacket();
  packet->status = STATUS_WAITING;
  return addCrc(packet);
}
 
#ifdef DEBUG  
void HomeNet::debugPacket(HomeNetPacket *packet)
{

  Serial.println("Decode Packet");
  Serial.print("    sent: ");
  Serial.println(packet->getStatus(),DEC);
  Serial.print("  length: ");
  Serial.println(packet->getLength(),DEC);
  Serial.print("settings: ");
  Serial.println(packet->getSettings(),DEC);
  Serial.print("    type: ");
  Serial.println(packet->getType(),DEC);
  Serial.print("fromNode: ");
  Serial.println(packet->getFromNode(),DEC);
  Serial.print("fromDevice: ");
  Serial.println(packet->getFromDevice(),DEC);
  Serial.print("  toNode: ");
  Serial.println(packet->getToNode(),DEC);
  Serial.print("  toDevice: ");
  Serial.println(packet->getToDevice(),DEC);
  //Serial.print("     ttl: ");
  //Serial.println(packet->ttl(),DEC);
  Serial.print("      id: ");
  Serial.println(packet->getId(),DEC);
  Serial.print(" command: ");
  Serial.println(packet->getCommand(),DEC);
  Serial.print(" payload: ");
  for(uint8_t i=0; i < packet->getPayloadLength(); i++){
    Serial.print(packet->getPayloadAt(i),DEC);
    Serial.print(",");
  } 
  Serial.println("");
  Serial.print(" payload: ");
  for(uint8_t i=0; i < packet->getPayloadLength(); i++){
    Serial.print(packet->getPayloadAt(i),BYTE);
  } 
  Serial.println("");
  Serial.print("checksum: ");
  Serial.println(packet->getChecksum(),DEC);
  Serial.print("fromPort: ");
  Serial.println(packet->getFromPort(),DEC);
}
#endif

//private
uint8_t HomeNet::_getId()
{
  return _uniqueId++; //loops when it overflows
}

int HomeNet::availableMemory() {
  int size = 2048; // Use 2048 with ATmega328
  byte *buf;

  while ((buf = (byte *) malloc(--size)) == NULL)
    ;

  free(buf);

  return size;
}

HomeNetPacket* HomeNet::addCrc(HomeNetPacket *packet){
    uint16_t dataChecksum;
    uint8_t i;
    for (i = 0; i < (packet->getLength()-OFFSET_FOOTER); i++) {
        dataChecksum = _crc16_update(dataChecksum, packet->data[i]);
    }
    packet->data[i++] = (dataChecksum >> 8) & 0xFF;
    packet->data[i] = dataChecksum & 0xFF;
    return packet;

}

void HomeNet::deviceUpdate() {

    for (uint8_t i = 0; i < _deviceCount; i++) {
        _devices[i]->updateOften();
    }

    if (millis() > (_deviceTimer + DEVICE_UPDATE)) {
        for (uint8_t i = 0; i < _deviceCount; i++) {
            _devices[i]->update();
        }
        _scheduleCount++;
        _deviceTimer = millis();
    }
}

void HomeNet::registerSchedule(HomeNetSchedule s[], uint8_t count ) { _deviceScheduleCount = count; _deviceSchedule = s; }
void HomeNet::registerInterrupts(HomeNetInterrupt i[], uint8_t count ) {_deviceInterruptCount = count; _deviceInterrupts = i; }

void HomeNet::deviceSchedule() {
    //_scheduleCount++;
    if (_scheduleCount == DEVICE_SCHEDULE) {
        //PayloadBuffer buffer;
        for (uint8_t i = 0; i < _deviceScheduleCount; i++) {
            if (_deviceSchedule[i].delay == 0) {
#ifdef DEBUG   
        Serial.print("Device Schedule Called on port ");
        Serial.println(i, DEC);
#endif
#ifdef DEBUG
           /*     Serial.print("Pre Send Payload: ");
                Serial.print("Command: ");
                Serial.print(_deviceSchedule[i].command, DEC);
                debugPayload(_deviceSchedule[i].payload);*/
#endif
                HomeNetPartialPacket* partial = _deviceSchedule[i].device->schedule(_deviceSchedule[i].command, _deviceSchedule[i].payload);
                if (partial != NULL) {
                    //check to make sure there is space in the packet
                    // if((partial->payload->length + buffer.length())> 56){
                    // if(packetType == 2){

#ifdef DEBUG
                    //Serial.print("Partial Payload: ");
                    //Serial.println(partial->fromDevice,DEC);
                    //debugPayload(partial->payload);
#endif

                    addUdpPacket(partial->fromDevice, _deviceSchedule[i].toNode, _deviceSchedule[i].toDevice, partial->command, partial->payload);
                    delete partial;
                }
                //*/
                _deviceSchedule[i].delay = _deviceSchedule[i].frequency - 1;

            } else {
                _deviceSchedule[i].delay--;
            }
        }
        _scheduleCount = 0;
    }
}

void HomeNet::deviceInterrupt(boolean run) {

    if ((HomeNetInterruptCount > 0) || run) {
#ifdef DEBUG
        Serial.print(HomeNetInterruptCount, DEC);
    Serial.println(" interrupt triggered");
#endif


        HomeNetInterruptCount = 0;
        for (uint8_t i = 0; i < _deviceInterruptCount; i++) {

            HomeNetPartialPacket* partial = _deviceInterrupts[i].device->interrupt(_deviceInterrupts[i].command, _deviceInterrupts[i].payload);
#ifdef DEBUG
            Serial.println("Interrupt Returned");
#endif
            if (partial != NULL) {
#ifdef DEBUG
                Serial.print("FromDevice: ");
                Serial.println(_deviceInterrupts[i].toDevice, DEC);
                Serial.print("Command: ");
                Serial.println(_deviceInterrupts[i].command, DEC);
                Serial.print("Partial Payload: ");
                //Serial.println(partial->fromDevice,DEC);
                debugPayload(partial->payload);
#endif
                addUdpPacket(partial->fromDevice, _deviceInterrupts[i]. toNode, _deviceInterrupts[i].toDevice, partial->command, partial->payload);
                delete partial;
            }
        }
  }
}

void HomeNet::loop() {
    //receive incoming packets
    receive();
    process();

    deviceUpdate();

    //still working on how often to call these
    receive();
    process();

    deviceSchedule();

    receive();
    process();

    deviceInterrupt();
}

void HomeNet::sleep(uint8_t mode) {

    //only sleep if stack is empty
    if (process() == false) {

       // uint16_t time = (_deviceTimer + DEVICE_UPDATE) - millis();
#ifdef DEBUG
//Serial.print("Entering Sleep for ");
//Serial.println(1000, DEC);

#endif
//delay(100); // these delays were part of the debug but they seem to be required ?!?
        //mode 0 idle for serial port wakeup

        //mode 5 powerdown allows rf12 to wakeup
        Sleepy::loseSomeTime(1000);//time);
        //delay(2000);
  #ifdef DEBUG
//Sleepy::watchdogInterrupts(-1);
Serial.println("Waking up");
//Serial.print("watchdog count: ");
//Serial.println(watchdogCounter, DEC);
//delay(20);

#endif
delay(5);
//statusLights->ledB(1);
//delay(100);
//statusLights->ledB(0);

    }
}

uint16_t  HomeNet::getDeviceId(uint8_t id) {
      if(id <= _deviceCount){
        return _devices[id]->getId();
    } else {
        return 0;
    }
}

//Serial Port///////////////////////////////////////////////////////////////////

void HomeNetPortSerial::init(uint8_t id)
{
    _sending = false;
    _receiving = false;
    _id = id;
    _receivingChecksum = 0;
    _ptr = 0;
    Serial.begin(SERIAL_SPEED);
    #ifdef DEBUG
    Serial.println("Serial Initialized");

     #endif
}

void HomeNetPortSerial::send(HomeNetPacket* packet) {
    if(_sending == false){
      _sending = true;
      //copy pointer
      _sendingRetryCount = 0;
    }
    _sendingTimer = millis();
    _sendingPacket = packet;

#ifdef DEBUG
    Serial.println("SENDING PACKET SERIAL");
#endif
    for (uint8_t i = 0; i < packet->getLength(); i++) {
        Serial.print(packet->data[i], BYTE);
    }
    packet->status = STATUS_SENT;
}

void HomeNetPortSerial::receive() {
    
    while (Serial.available() > 0) { 
        uint8_t byteIn;
        byteIn = Serial.read();
        if (_ptr == 0) { //new byte in

            if (byteIn == 0) { //resend last
                if (_sending == true) {
                    _sendingPacket->status = STATUS_SENDING;
                }
                return; 
            }
            if (byteIn == 255) { //aknowledge
                if (_sending == true) {
                    _sendingPacket->status = STATUS_SUCCESS;
                    _sending = false; //allow next packet to send
                    _ptr = 0;
                }
                return;
            }

            if ((byteIn < 10) || (byteIn > 66)) { //bad byte, send 0 to start over
                Serial.flush();
                Serial.print(3, BYTE);
                return;
            }

            _receiving = true;
            _receivingTimer = millis();
            _receivingPacket = _homeNet->getNewPacket();
            _receivingPacket->fromPort = _id;
            _receivingPacket->setLength(byteIn);
            _receivingPacket->status = STATUS_RECEVING;
            _receivingChecksum = 0;
        }
        buildPacket(byteIn);
    }

    //process_stack();
    
    if ((_receiving == true) && ((millis() - _receivingTimer) > PACKET_TIMEOUT)) {

#ifdef DEBUG
    Serial.println("Recieving Packet Timed Out");
#endif
        _receivingPacket = STATUS_CLEAR;
        _receiving = false;
  
        //Serial.flush();
        Serial.print(2, BYTE); //tell the node to resend last
    }
    //since this loops every cycle
    //might be better to move this to it's own function so it clearer
    //check for packets that are taking too long to veryify that they sent

    
    if ((_sending == true) && ((millis() - _sendingTimer) > PACKET_TIMEOUT)) {

#ifdef DEBUG
    Serial.println("Sending Packet Timed Out");
    //Serial.print("Time Out: ");
    // Serial.print((millis() - _sendingTimer),DEC);
#endif
       _sendingRetryCount++;
        if (_sendingRetryCount <= PACKET_RETRY) {

#ifdef DEBUG
    Serial.print("Timeout Count: ");
     Serial.println(_sendingRetryCount,DEC);
#endif
            _sendingPacket->status = STATUS_SENDING;

        } else {
            _sendingPacket->status = STATUS_FAILED;
            _sending = false;
        }
    }
}

void HomeNetPortSerial::buildPacket(uint8_t byteIn) {
    if (_ptr < (_receivingPacket->getLength() - OFFSET_FOOTER)) { //add to check sum
        _receivingPacket->data[_ptr] = byteIn;
        _receivingChecksum = _crc16_update(_receivingChecksum, byteIn);
        _ptr++;
        return;
    } else if (_ptr == (_receivingPacket->getLength() - OFFSET_FOOTER)) { //get first byte of checksum
        _receivingPacket->data[_ptr] = byteIn;
        _ptr++;
        return;
    } else { //get second byte and check the checksum
        uint16_t checksum = _receivingPacket->data[_ptr-1];
        _receivingPacket->data[_ptr] = byteIn;
        checksum = (checksum << 8) | byteIn;

        if (_receivingChecksum != checksum) {
            //packet failed crc check
            _receivingPacket->status = STATUS_CLEAR;
            Serial.flush();
            Serial.print(0, BYTE); //tell the node to resend last
        } else {
            
            _receivingPacket->status = STATUS_RECEIVED;
            //Serial.flush();
            Serial.print(255, BYTE); //tell the node the packet was successful
        }

        //reset incomingPacket

    }
    
    _receiving = false;
    _ptr = 0;
}


//RF12//////////////////////////////////////////////////////////////////////////

void HomeNetPortRF12::init(uint8_t id) {
#ifdef DEBUG
    Serial.println("RF12 Initialized");
#endif
    _sending = false;
    _receiving = false;
    _id = id;
    _sleeping =false;


    //_mode = 0;
    if (_homeNet->_nodeId <= 32) {
        rf12_initialize(char(_homeNet->_nodeId), _freq, _group);
        if(_mode == 1){
            rf12_sleep(0);
            _sleeping = true;
        }
    }
#ifdef DEBUG
    //else {
    //    Serial.println("Invalid node id for RF12");
    // }
#endif
}

void HomeNetPortRF12::send(HomeNetPacket* packet) {
    _sending = true;
    if((_mode == 1) && (_sleeping == true)){
        Serial.println("Waking Up rf12");
        rf12_sleep(-1);
        delay(5); //wakeup
        _sleeping = false;
    }

    if (rf12_canSend()) { //check if radio is free
#ifdef DEBUG
        Serial.println("SENDING PACKET RF12");
        //_homeNet->debugPacket(packet);
Serial.println("Packet:");
         for (uint8_t i = 0; i < packet->getLength(); i++) {
        Serial.print(packet->data[i], DEC);
        Serial.print(",");
        }

Serial.println("");

#endif
        byte header = (packet->getType() == PACKET_TCP) ? RF12_HDR_ACK : 0;
        //Serial.print("Node Header: ");
        //Serial.println(header,DEC);
        byte toNode = packet->getToNode();
        //Serial.print("Send to Node: ");
        //Serial.println(toNode,DEC);

        if (toNode > 0) {
            if (toNode > 32) {
                toNode = RF12_NODE_OUT_OF_RANGE_SEND_TO; //if port is too high send to basestation to retransmit
            }
            header |= RF12_HDR_DST | toNode;
        }
        //rf12 does it's own check sum. no need to waste resources doing it twice
        rf12_sendStart(header, packet->data, packet->getLength() - OFFSET_FOOTER);

        if (_homeNet->statusLights != NULL) {
            _homeNet->statusLights->ledA(2);
        }
        if((packet->getType() != 1) && (_mode == 1)){ //if it didn't just send a tcp packet, return to sleep, else, listen for the ack
            delay(5);
            rf12_sleep(0);
            _sleeping = true;
        } else {
            _waiting = true;
        }

        packet->status = STATUS_SUCCESS;
        _sending = false;
    }
}

void HomeNetPortRF12::receive() {
    if ((_waiting == true) || (_sleeping == false)) {
        if (rf12_recvDone() && (rf12_crc == 0)) {

#ifdef DEBUG
            Serial.println("RF12 Receiving");
#endif

            if ((rf12_hdr & RF12_HDR_ACK) && !(rf12_hdr & RF12_HDR_CTL) && (_homeNet->_nodeId & COLLECT) == 0) {
                byte addr = rf12_hdr & RF12_HDR_MASK;
                // if request was sent only to us, send ack back as broadcast
                rf12_sendStart(rf12_hdr & RF12_HDR_DST ? RF12_HDR_CTL : RF12_HDR_CTL | RF12_HDR_DST | addr, 0, 0);
            }
            //HomeNetPacket *p = receivePacketRf12((void*)rf12_data,rf12_len);

            //HomeNetPacket* HomeNet::receivePacketRf12(const void* payload, uint8_t length)

            HomeNetPacket* packet = _homeNet->getNewPacket();
            memcpy(packet->data, (void*) rf12_data, rf12_len);
            packet->fromPort = _id;
            packet->status = STATUS_RECEIVED;
            _homeNet->addCrc(packet);

            //return packet;

            //blink led
            if (_homeNet->statusLights != NULL) {
                _homeNet->statusLights->ledB(2);
            }

            //@todo check to see if the packet matches ack, if we did, turn off the radio
            if ((_mode == 1)) {
                rf12_sleep(0);
                _sleeping = true;
            }
        }
    }
}

ISR(WDT_vect) { Sleepy::watchdogEvent(); }