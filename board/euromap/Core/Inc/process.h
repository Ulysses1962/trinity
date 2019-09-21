#include "main.h"

#ifndef __PROCESS_H__
#define __PROCESS_H__
extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim6;

#define COMMAND_TIMEOUT 250

/**
//==============================================================================
// MACHINE COMMANDS
//==============================================================================
    1. E000      - device op mode enable
    2. E001      - device op mode disable
    3. E002      - start handling device operation cycle 
    4. E003      - finish handling device operation cycle 
    5. E004      - set device emergency condition
    6. E005      - drop device emergency condition
    7. E006      - move ejector in forward position
    8. E007      - move ejector in back position
    9. E008      - enable moulding removal (set core pullers to position 2)
    A. E009      - prepare next moulding cycle (set core pullers to position 1)
    B. E010      - enabling mould closure
    C. E011      - disabling mould closure
    D. E012      - set mould area free
    E. E013      - set mould area occupied
*/

void EmapCommandRead(void);
void EmapCommandParse(void);
void onEmapCommandReceived(void);
bool EmapIsCommandReceived(void);
uint8_t EmapGetCommandCode(void);
void EmapCommandProcessing(void);
void EmapInit(void);
static void EmapCommandAck(bool cmd_result);
static void EmapStateCheck(void); 

#ifdef MOULDING_TEST_MODE
static void EmapSendState(void);
#endif

static void E000(void);
static void E001(void);
static void E002(void);
static void E003(void);
static void E004(void);
static void E005(void);
static void E006(void);
static void E007(void);
static void E008(void);
static void E009(void);
static void E010(void);
static void E011(void);
static void E012(void);
static void E013(void);

#endif
