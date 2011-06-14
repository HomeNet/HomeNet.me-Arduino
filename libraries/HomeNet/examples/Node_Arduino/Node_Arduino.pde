// Testing changes to Payload fns
// 12-13-2010 <mdoll@email.usf.edu> http://opensource.org/licenses/mit-license.php

#include <Ports.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>

//Start HomeNet packet stack
HomeNet stack(0x05);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortSerial portSerial(stack, PORT_SERIAL);  //PORT_SERIAL tells the adpater which serial serial port to use (for future support of the arduino mega)

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);

//Package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portSerial};
HomeNetDevice * devices[] = {&jeeNode, &statusLights};

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
//HomeNetSchedule schedule[] = {{5,10,&statusLights,255,0,CMD_GETVALUE,0},
//                              {0,10,&statusLights,255,0,CMD_GETVALUE,0}};                              

void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

MilliTimer sendTimer;

void loop() {
  //Receive incoming packets
  stack.receive();
  stack.process();

  stack.deviceUpdate();
  
  if (sendTimer.poll(10000)){
    long number = 133463623;
    stack.debugPayload(HomeNetPayload(number));
  }   

  //Still working on how often to call these
  stack.receive();
  stack.process();

  stack.deviceInterrupt();
 // stack.deviceSchedule();
}


