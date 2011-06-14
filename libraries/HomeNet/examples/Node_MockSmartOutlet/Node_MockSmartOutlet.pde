// Code for SmartOutlet Mockup
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//Start HomeNet packet stack
HomeNet stack(0x05);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33);

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceSmartOutlet outlet(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &outlet};

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
HomeNetSchedule schedule[] = {{ 0,10,&outlet,255,0,CMD_ON, HomeNetPayload((uint8_t) 0)},
                              { 5,10,&outlet,255,0,CMD_OFF,HomeNetPayload((uint8_t) 0)},
                              { 5,10,&outlet,255,0,CMD_ON, HomeNetPayload((uint8_t) 1)},
                              {10,10,&outlet,255,0,CMD_OFF,HomeNetPayload((uint8_t) 1)}};

void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}
