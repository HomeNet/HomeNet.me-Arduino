// Test of Motion Sensor Interrupt.
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//Start HomeNet Packet Stack
HomeNet stack(0x02);//0x01 is RF12 base station //0xFF is PC uplink

//Setup Network Adapters
HomeNetPortRF12   portRF12(stack, SEND_RECEIVE, RF12_915MHZ, 33);

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceMotionSensor motion(stack, 20); //timeout

//package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &motion}; 

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
//HomeNetSchedule schedule[] = {{5,10,&tmp37,4,2,CMD_GETVALUE,0},
//                              {0,10,&ldr,4,2,CMD_GETVALUE,0}};
                              
HomeNetInterrupt interrupt[] = {{&motion,6,2,CMD_GETVALUE,0}};  

void setup() {
  //initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
  stack.registerInterrupts(interrupt,sizeof(interrupt)/sizeof(interrupt[0]));

}

void loop() {
  stack.loop();
  //stack.sleep(); //coming soon
}
