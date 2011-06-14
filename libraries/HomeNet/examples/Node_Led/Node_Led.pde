// Simple Test of LED. Only p2 and p3 support PWM (analog write)
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//Start HomeNet packet stack
HomeNet stack(0x03);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33); //only stack is required, see HomeNetConfig.h to set defaults

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceLED led(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &led}; 
 
//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
/*HomeNetSchedule schedule[] = {{0,8,&led,255,0,CMD_ON, 0},
                              {2,8,&led,255,0,CMD_OFF,0},
                              {4,8,&led,255,0,CMD_LEVEL,Payload((uint8_t) 20)},
                              {6,8,&led,255,0,CMD_LEVEL,Payload((uint8_t) 120)}}; */
void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


