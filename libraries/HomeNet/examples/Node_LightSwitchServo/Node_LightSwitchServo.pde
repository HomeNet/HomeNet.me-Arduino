// Control A light switch with a servo
// 12-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Servo.h> 

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

class HomeNetDeviceLightSwitchServo : public HomeNetDevice {

public:
    HomeNetDeviceLightSwitchServo(HomeNet& homeNet ):HomeNetDevice(homeNet) {}
    inline uint16_t getId(){ return 18; }
    void init(uint8_t location){ 
      _location = location;
      _currentPosition = 0;
      _position = 0;
      _servo = new Servo;
      _servo->attach(5);
    }
    HomeNetPartialPacket* process(const uint16_t& fromNode, const uint8_t& fromDevice, const uint8_t& command, const HomeNetPayload& payload){

      switch(command){
  
          case CMD_GETVALUE: 
              return partialPacket(_location, CMD_BYTE, HomeNetPayload(_position));
              break; 
          case CMD_BYTE:
          case CMD_SETVALUE: 
              if(payload.data[0] >128){
                _position = 120;  
              } else {
                _position = 60;
              }
              break; 
  
           case CMD_ON:
                _position = 120;
              break;
           case CMD_OFF:
                _position = 60;
              break;
      }
      return NULL;
    }

  void updateOften(){
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
    
  
private:
   Servo * _servo;
   uint8_t _currentPosition;
   uint8_t _position;
};


//Start HomeNet Packet Stack
HomeNet stack(0x07);//0x01 is RF12 base station //0xFF is PC uplink

//Setup Network Adapters
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33);

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceLightSwitchServo switchServo(stack);


//package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &switchServo}; 

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
HomeNetSchedule schedule[] = {{ 0,20,&switchServo,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t) 255)},
                              {10,20,&switchServo,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t) 0)}};

void setup() {
  //initize home net with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 

  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


