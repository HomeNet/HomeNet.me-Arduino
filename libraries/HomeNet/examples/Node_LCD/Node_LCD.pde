// Test of LCD Device.
// 01-15-2011 <mdoll@mail.usf.edu> http://opensource.org/licenses/mit-license.php

#include <PortsLCD.h>
#include <RF12.h>
#include <HomeNet.h>
#include <HomeNetDevices.h>
#include <HomeNetDeviceLCD.h>

//Start HomeNet packet stack
HomeNet stack(0x04);//0x01 is RF12 base station //0xFF is PC uplink

//Setup network adapters
HomeNetPortRF12   portRF12(stack);

//Setup attached devices
HomeNetDeviceJeeNode jeeNode(stack);
HomeNetDeviceStatusLights statusLights(stack);
HomeNetDeviceLCD lcd(stack);

//package the setup info in a nice neat arrays
HomeNetPort * ports[] = {&portRF12};
HomeNetDevice * devices[] = {&jeeNode, &statusLights, &lcd};

//2,2 = TMP37
//2,3 = LDR
//line, fromNode, fromDevice, Message
HomeNetDeviceLCD::Display screen[] ={{0,2,2,"Temp: %sF"},
                                     {1,2,3,"Brightness: %s%%"}};

//delay (sec), frequency (sec), device, sendToNode, sendToDevice, Command, Payload
//HomeNetSchedule schedule[] = {{5,10,&lcd,255,0,CMD_SETVALUE,HomeNetPayload("Test 1")},
//                              {0,10,&lcd,255,0,CMD_SETVALUE,HomeNetPayload("Test 2")}};
                               
void setup() {
  //Initialize HomeNet with the setup info
  stack.init(ports, sizeof(ports)/2, devices, sizeof(devices)/2); 
  lcd.registerDisplay(screen, sizeof(screen)/sizeof(screen[0]));
  stack.registerStatusLights(statusLights); //setup status lights
  //stack.registerSchedule(schedule,sizeof(schedule)/sizeof(schedule[0]));
}

void loop() {
  stack.loop();
}


