#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=avr-gcc.exe
CCC=avr-g++.exe
CXX=avr-g++.exe
FC=
AS=avr-as.exe

# Macros
CND_PLATFORM=WinAVR-Windows
CND_CONF=Release
CND_DISTDIR=dist

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1730660635/PortsBMP085.o \
	${OBJECTDIR}/_ext/546314031/LibHumidity.o \
	${OBJECTDIR}/HomeNetDevicesButton.o \
	${OBJECTDIR}/HomeNetDevices.o \
	${OBJECTDIR}/_ext/156170367/LibHumidity.o \
	${OBJECTDIR}/_ext/82737666/RF12.o \
	${OBJECTDIR}/_ext/1730660635/Ports.o \
	${OBJECTDIR}/_ext/156170178/LibHumidity.o \
	${OBJECTDIR}/_ext/82737666/RF12sio.o \
	${OBJECTDIR}/HomeNet.o \
	${OBJECTDIR}/PortsSHT21.o \
	${OBJECTDIR}/_ext/1730660635/PortsLCD.o \
	${OBJECTDIR}/_ext/156171526/PortsSHT21.o \
	${OBJECTDIR}/_ext/1730660635/PortsSHT11.o \
	${OBJECTDIR}/_ext/1730660635/PortsRF12.o \
	${OBJECTDIR}/HomeNetDeviceLCD.o \
	${OBJECTDIR}/_ext/666785897/LibHumidity.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-Release.mk dist/Release/WinAVR-Windows/homenet.exe

dist/Release/WinAVR-Windows/homenet.exe: ${OBJECTFILES}
	${MKDIR} -p dist/Release/WinAVR-Windows
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/homenet ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/_ext/1730660635/PortsBMP085.o: C/mp/arduino-0021/libraries/Ports/PortsBMP085.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1730660635
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1730660635/PortsBMP085.o C/mp/arduino-0021/libraries/Ports/PortsBMP085.cpp

${OBJECTDIR}/_ext/546314031/LibHumidity.o: ../LibHumidity/old2/LibHumidity.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/546314031
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/546314031/LibHumidity.o ../LibHumidity/old2/LibHumidity.cpp

${OBJECTDIR}/HomeNetDevicesButton.o: HomeNetDevicesButton.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/HomeNetDevicesButton.o HomeNetDevicesButton.cpp

${OBJECTDIR}/HomeNetDevices.o: HomeNetDevices.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/HomeNetDevices.o HomeNetDevices.cpp

${OBJECTDIR}/_ext/156170367/LibHumidity.o: ../LibHumidity/old/LibHumidity.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/156170367
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/156170367/LibHumidity.o ../LibHumidity/old/LibHumidity.cpp

${OBJECTDIR}/_ext/82737666/RF12.o: C/mp/arduino-0021/libraries/RF12/RF12.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/82737666
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/82737666/RF12.o C/mp/arduino-0021/libraries/RF12/RF12.cpp

${OBJECTDIR}/_ext/1730660635/Ports.o: C/mp/arduino-0021/libraries/Ports/Ports.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1730660635
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1730660635/Ports.o C/mp/arduino-0021/libraries/Ports/Ports.cpp

${OBJECTDIR}/_ext/156170178/LibHumidity.o: ../LibHumidity/org/LibHumidity.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/156170178
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/156170178/LibHumidity.o ../LibHumidity/org/LibHumidity.cpp

${OBJECTDIR}/_ext/82737666/RF12sio.o: C/mp/arduino-0021/libraries/RF12/RF12sio.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/82737666
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/82737666/RF12sio.o C/mp/arduino-0021/libraries/RF12/RF12sio.cpp

${OBJECTDIR}/HomeNet.o: HomeNet.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/HomeNet.o HomeNet.cpp

${OBJECTDIR}/PortsSHT21.o: PortsSHT21.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/PortsSHT21.o PortsSHT21.cpp

${OBJECTDIR}/_ext/1730660635/PortsLCD.o: C/mp/arduino-0021/libraries/Ports/PortsLCD.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1730660635
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1730660635/PortsLCD.o C/mp/arduino-0021/libraries/Ports/PortsLCD.cpp

${OBJECTDIR}/_ext/156171526/PortsSHT21.o: ../LibHumidity/new/PortsSHT21.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/156171526
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/156171526/PortsSHT21.o ../LibHumidity/new/PortsSHT21.cpp

${OBJECTDIR}/_ext/1730660635/PortsSHT11.o: C/mp/arduino-0021/libraries/Ports/PortsSHT11.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1730660635
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1730660635/PortsSHT11.o C/mp/arduino-0021/libraries/Ports/PortsSHT11.cpp

${OBJECTDIR}/_ext/1730660635/PortsRF12.o: C/mp/arduino-0021/libraries/Ports/PortsRF12.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1730660635
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/1730660635/PortsRF12.o C/mp/arduino-0021/libraries/Ports/PortsRF12.cpp

${OBJECTDIR}/HomeNetDeviceLCD.o: HomeNetDeviceLCD.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/HomeNetDeviceLCD.o HomeNetDeviceLCD.cpp

${OBJECTDIR}/_ext/666785897/LibHumidity.o: ../LibHumidity/LibHumidity.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/666785897
	${RM} $@.d
	$(COMPILE.cc) -O2 -MMD -MP -MF $@.d -o ${OBJECTDIR}/_ext/666785897/LibHumidity.o ../LibHumidity/LibHumidity.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/Release
	${RM} dist/Release/WinAVR-Windows/homenet.exe

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
