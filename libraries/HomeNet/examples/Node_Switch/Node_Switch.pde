// Simple Test of Node Switch. right now only one switch can be attached, uses Interrupt 1
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
HomeNetDeviceSwitch lswitch(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};//,&portSerial,
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &lswitch}; 
 
//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
HomeNetInterrupt interrupt[] = {{&lswitch,7,2,CMD_GETVALUE,0}};

void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  stack.registerInterrupts(interrupt,sizeof(interrupt)/sizeof(interrupt[0]));
}

void loop() {
  stack.loop();
}


