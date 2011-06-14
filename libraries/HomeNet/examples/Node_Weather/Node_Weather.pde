// Test of TMP37 and LDR sensors.
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//Start HomeNet Packet Stack
HomeNet stack(0x08);//0x01 is RF12 base station //0xFF is PC uplink

//Setup Network Adapters
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33);

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceTMP421 tmp421(stack);
//HomeNetDeviceLight light(stack);

//package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &tmp421}; 

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
HomeNetSchedule schedule[] = {{5,10,&tmp421,255,0,CMD_GETVALUE,0}};//,
                             // {0,10,&light,255,0,CMD_GETVALUE,0}};
                               
void setup() {
  //initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}
