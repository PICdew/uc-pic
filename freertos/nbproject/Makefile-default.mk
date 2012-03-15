#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
include Makefile

# Environment
# Adding MPLAB X bin directory to path
PATH:=/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/:$(PATH)
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/main.o ${OBJECTDIR}/_ext/1585373384/croutine.o ${OBJECTDIR}/_ext/1585373384/list.o ${OBJECTDIR}/_ext/1585373384/queue.o ${OBJECTDIR}/_ext/1585373384/tasks.o ${OBJECTDIR}/_ext/1585373384/timers.o ${OBJECTDIR}/serial/serial.o ${OBJECTDIR}/port/port.o ${OBJECTDIR}/CmdInt/CommandInterpreter.o ${OBJECTDIR}/_ext/1578989285/heap_2.o
POSSIBLE_DEPFILES=${OBJECTDIR}/main.o.d ${OBJECTDIR}/_ext/1585373384/croutine.o.d ${OBJECTDIR}/_ext/1585373384/list.o.d ${OBJECTDIR}/_ext/1585373384/queue.o.d ${OBJECTDIR}/_ext/1585373384/tasks.o.d ${OBJECTDIR}/_ext/1585373384/timers.o.d ${OBJECTDIR}/serial/serial.o.d ${OBJECTDIR}/port/port.o.d ${OBJECTDIR}/CmdInt/CommandInterpreter.o.d ${OBJECTDIR}/_ext/1578989285/heap_2.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/main.o ${OBJECTDIR}/_ext/1585373384/croutine.o ${OBJECTDIR}/_ext/1585373384/list.o ${OBJECTDIR}/_ext/1585373384/queue.o ${OBJECTDIR}/_ext/1585373384/tasks.o ${OBJECTDIR}/_ext/1585373384/timers.o ${OBJECTDIR}/serial/serial.o ${OBJECTDIR}/port/port.o ${OBJECTDIR}/CmdInt/CommandInterpreter.o ${OBJECTDIR}/_ext/1578989285/heap_2.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

# Path to java used to run MPLAB X when this makefile was created
MP_JAVA_PATH="/usr/lib/jvm/java-6-openjdk/jre/bin/"
OS_CURRENT="$(shell uname -s)"
############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
MP_CC="/opt/microchip/mplabc18/v3.40/bin/mcc18"
# MP_BC is not defined
MP_AS="/opt/microchip/mplabc18/v3.40/bin/../mpasm/MPASMWIN"
MP_LD="/opt/microchip/mplabc18/v3.40/bin/mplink"
MP_AR="/opt/microchip/mplabc18/v3.40/bin/mplib"
DEP_GEN=${MP_JAVA_PATH}java -jar "/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/extractobjectdependencies.jar" 
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps
MP_CC_DIR="/opt/microchip/mplabc18/v3.40/bin"
# MP_BC_DIR is not defined
MP_AS_DIR="/opt/microchip/mplabc18/v3.40/bin/../mpasm"
MP_LD_DIR="/opt/microchip/mplabc18/v3.40/bin"
MP_AR_DIR="/opt/microchip/mplabc18/v3.40/bin"
# MP_BC_DIR is not defined

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F2455
MP_PROCESSOR_OPTION_LD=18f2455
MP_LINKER_DEBUG_OPTION= -u_DEBUGCODESTART=0x5dc0 -u_DEBUGCODELEN=0x240 -u_DEBUGDATASTART=0x3f4 -u_DEBUGDATALEN=0xb
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	
${OBJECTDIR}/_ext/1585373384/croutine.o: ../../freertos/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/croutine.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/croutine.o   ../../freertos/Source/croutine.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/croutine.o 
	
${OBJECTDIR}/_ext/1585373384/list.o: ../../freertos/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/list.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/list.o   ../../freertos/Source/list.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/list.o 
	
${OBJECTDIR}/_ext/1585373384/queue.o: ../../freertos/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/queue.o   ../../freertos/Source/queue.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/queue.o 
	
${OBJECTDIR}/_ext/1585373384/tasks.o: ../../freertos/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/tasks.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/tasks.o   ../../freertos/Source/tasks.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/tasks.o 
	
${OBJECTDIR}/_ext/1585373384/timers.o: ../../freertos/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/timers.o   ../../freertos/Source/timers.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/timers.o 
	
${OBJECTDIR}/serial/serial.o: serial/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/serial 
	@${RM} ${OBJECTDIR}/serial/serial.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/serial/serial.o   serial/serial.c 
	@${DEP_GEN} -d ${OBJECTDIR}/serial/serial.o 
	
${OBJECTDIR}/port/port.o: port/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/port 
	@${RM} ${OBJECTDIR}/port/port.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/port/port.o   port/port.c 
	@${DEP_GEN} -d ${OBJECTDIR}/port/port.o 
	
${OBJECTDIR}/CmdInt/CommandInterpreter.o: CmdInt/CommandInterpreter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/CmdInt 
	@${RM} ${OBJECTDIR}/CmdInt/CommandInterpreter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/CmdInt/CommandInterpreter.o   CmdInt/CommandInterpreter.c 
	@${DEP_GEN} -d ${OBJECTDIR}/CmdInt/CommandInterpreter.o 
	
${OBJECTDIR}/_ext/1578989285/heap_2.o: ../../freertos/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1578989285 
	@${RM} ${OBJECTDIR}/_ext/1578989285/heap_2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1578989285/heap_2.o   ../../freertos/Source/portable/MemMang/heap_2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1578989285/heap_2.o 
	
else
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/main.o   main.c 
	@${DEP_GEN} -d ${OBJECTDIR}/main.o 
	
${OBJECTDIR}/_ext/1585373384/croutine.o: ../../freertos/Source/croutine.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/croutine.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/croutine.o   ../../freertos/Source/croutine.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/croutine.o 
	
${OBJECTDIR}/_ext/1585373384/list.o: ../../freertos/Source/list.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/list.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/list.o   ../../freertos/Source/list.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/list.o 
	
${OBJECTDIR}/_ext/1585373384/queue.o: ../../freertos/Source/queue.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/queue.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/queue.o   ../../freertos/Source/queue.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/queue.o 
	
${OBJECTDIR}/_ext/1585373384/tasks.o: ../../freertos/Source/tasks.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/tasks.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/tasks.o   ../../freertos/Source/tasks.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/tasks.o 
	
${OBJECTDIR}/_ext/1585373384/timers.o: ../../freertos/Source/timers.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1585373384 
	@${RM} ${OBJECTDIR}/_ext/1585373384/timers.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1585373384/timers.o   ../../freertos/Source/timers.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1585373384/timers.o 
	
${OBJECTDIR}/serial/serial.o: serial/serial.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/serial 
	@${RM} ${OBJECTDIR}/serial/serial.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/serial/serial.o   serial/serial.c 
	@${DEP_GEN} -d ${OBJECTDIR}/serial/serial.o 
	
${OBJECTDIR}/port/port.o: port/port.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/port 
	@${RM} ${OBJECTDIR}/port/port.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/port/port.o   port/port.c 
	@${DEP_GEN} -d ${OBJECTDIR}/port/port.o 
	
${OBJECTDIR}/CmdInt/CommandInterpreter.o: CmdInt/CommandInterpreter.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/CmdInt 
	@${RM} ${OBJECTDIR}/CmdInt/CommandInterpreter.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/CmdInt/CommandInterpreter.o   CmdInt/CommandInterpreter.c 
	@${DEP_GEN} -d ${OBJECTDIR}/CmdInt/CommandInterpreter.o 
	
${OBJECTDIR}/_ext/1578989285/heap_2.o: ../../freertos/Source/portable/MemMang/heap_2.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1578989285 
	@${RM} ${OBJECTDIR}/_ext/1578989285/heap_2.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) --verbose -w3 -I"/projects/freertos/Source/include" -I"/projects/freertos/Demo/Common/include" -I"/projects/pic/freertos/port" -I"/projects/freertos/Demo/Common/Utils" -Ls  -I ${MP_CC_DIR}/../h  -fo ${OBJECTDIR}/_ext/1578989285/heap_2.o   ../../freertos/Source/portable/MemMang/heap_2.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/1578989285/heap_2.o 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "18f2455.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG   -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}/../lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "18f2455.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w    -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}/../lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/freertos.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "/opt/microchip/mplabx/mplab_ide/mplab_ide/modules/../../bin/"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
