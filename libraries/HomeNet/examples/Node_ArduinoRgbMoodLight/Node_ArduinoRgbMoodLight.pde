// Simple Test of LED. Only p2 and p3 support PWM (analog write)
// 11-29-2010 <mdoll@usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//http://blog.appliedplatonics.com/?p=9
RGB hsv_to_rgb(float h, float s, float v) {
  
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

class HomeNetDeviceRgbMoodLight : public HomeNetDevice {

public:
    HomeNetDeviceRgbMoodLight(HomeNet& homeNet ):HomeNetDevice(homeNet) {}
    inline uint16_t getId(){ return 17; }
    void init(uint8_t location){ 
      _location = location; 
      pinMode(9, OUTPUT); 
      pinMode(10, OUTPUT); 
      pinMode(11, OUTPUT); 
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
}
    HomeNetPartialPacket* process(const uint8_t& command, const HomeNetPayload& payload){
    switch(command){

        case CMD_GETVALUE: 
            return partialPacket(_location, CMD_BYTE, HomeNetPayload(_hue));
            break; 
         case CMD_BYTE:
         case CMD_SETVALUE:
             setColor(payload.data[0]);
            break;
         case CMD_ON:
              digitalWrite(9,LOW);
              digitalWrite(10,LOW);
              digitalWrite(11,LOW);
            break;
         case CMD_OFF:
               digitalWrite(9,HIGH);
              digitalWrite(10,HIGH);
              digitalWrite(11,HIGH);
            break;
    }
    return NULL;
}
    
  void setColor(uint8_t level)  { 
    int offset = 200;
    int hue = map(level,0,255,0,360); 
     
    hue += offset;
    if(hue > 360){
      hue -= 360; 
    } else if ( hue < 0){
      hue += 360; 
    }
    _hue = hue;
    RGB color = hsv_to_rgb(_hue,1,.8);
    analogWrite(9 ,color.r); 
    analogWrite(10,color.g); 
    analogWrite(11,color.b); 
  }

private:
   uint8_t _hue;
};


//Start HomeNet Packet Stack
HomeNet stack(0x40);//0x01 is RF12 base station //0xFF is PC uplink

//Setup Network Adapters
HomeNetPortSerial portSerial(stack, PORT_SERIAL);  //PORT_SERIAL tells the adpater which serial serial port to use (for future support of the arduino mega)

//Setup attached devices
HomeNetDeviceArduino arduino(stack);
HomeNetDeviceRgbMoodLight mood(stack);

//package the setup info in a nice neat arrays
 HomeNetPort * ports[] = {&portSerial};
 HomeNetDevice * devices[] = {&arduino, &mood}; 

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
   HomeNetSchedule schedule[] = {{0,8,&mood,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,0,0},3)},
                                 {2,8,&mood,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {255,0,0},3)},
                                 {4,8,&mood,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,255,0},3)},
                                 {6,8,&mood,255,0,CMD_SETVALUE,HomeNetPayload((uint8_t[]) {0,0,255},3)}};

void setup() {
  //initize home net with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


