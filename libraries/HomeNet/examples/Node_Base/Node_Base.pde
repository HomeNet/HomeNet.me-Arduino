// Example of a HomeNet RF12 Base Station
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>
 
//Start HomeNet packet stack
HomeNet stack(0x01);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortSerial portSerial(stack, PORT_SERIAL);  //PORT_SERIAL tells the adpater which serial serial port to use (for future support of the arduino mega)
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33); //only stack is required, see HomeNetConfig.h to set defaults

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portSerial, &portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights};

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
//HomeNetSchedule schedule[] = {{5,10,&jeeNode,255,0,CMD_MEMORYFREE,0}};                         
                               
void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


