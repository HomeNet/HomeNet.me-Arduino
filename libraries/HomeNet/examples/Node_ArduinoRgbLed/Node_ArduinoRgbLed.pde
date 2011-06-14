// RGB Lamp based on an arduino using pins 9-11 for RGB
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

class HomeNetDeviceRgbLed : public HomeNetDevice {

public:
    HomeNetDeviceRgbLed(HomeNet& homeNet ):HomeNetDevice(homeNet) {}
    inline uint16_t getId(){ return 16; }
    void init(uint8_t location){ 
      _location = location; 
      _red = 0;
      _green = 0;
      _blue = 0;
      pinMode(9, OUTPUT); 
      pinMode(10, OUTPUT); 
      pinMode(11, OUTPUT); 
      digitalWrite(9,HIGH);
      digitalWrite(10,HIGH);
      digitalWrite(11,HIGH);
}
    HomeNetPartialPacket* process(const uint8_t& command, const HomeNetPayload& payload){
    switch(command){

        case CMD_GETVALUE: {
            PayloadBuffer buffer;
            buffer.print(_red);
            buffer.print(_green);
            buffer.print(_blue);
            return partialPacket(_location, CMD_BYTE, buffer.payload());
            break; }
         case CMD_BYTE:
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
                    red(255);
                } else if(payload.data[0] == 1) {
                    green(255);
                } else {
                    blue(255);
                }
            }
            break;
    }
    return NULL;
}
    
    void red(uint8_t level)  { _red  =level; analogWrite(9 ,255-level); }
    void green(uint8_t level){ _green=level; analogWrite(10,255-level); }
    void blue(uint8_t level) { _blue =level; analogWrite(11,255-level); }

private:
   uint8_t _red;
   uint8_t _green;
   uint8_t _blue;
};

//Start HomeNet packet stack
HomeNet stack(0x40);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortSerial portSerial(stack, PORT_SERIAL);  //PORT_SERIAL tells the adpater which serial serial port to use (for future support of the arduino mega)

//Setup attached devices
HomeNetDeviceArduino arduino(stack);
HomeNetDeviceRgbLed led(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portSerial};
HomeNetDevice * devices[] = {&arduino, &led}; 

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
HomeNetSchedule schedule[] = {{0,8,&led,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,0,0},3)},
                              {2,8,&led,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {255,0,0},3)},
                              {4,8,&led,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,255,0},3)},
                              {6,8,&led,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,0,255},3)}};

void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  //stack.registerStatusLights(statusLights); //setup status lights
  stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


