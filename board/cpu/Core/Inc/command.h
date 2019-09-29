#include "main.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdbool.h"


#ifndef __COMMAND_H__
#define __COMMAND_H__

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/** ------------------------------------------------------------------------------------------------
    COMMAND LIST
    ------------------------------------------------------------------------------------------------
    A. AXIS MOVEMENT COMMANDS
    ------------------------------------------------------------------------------------------------
    1. G0000:+000.000 - move along X-axis in specified direcrion on set distance
    2. G0001:+000.000 - move along Y-axis in specified direcrion on set distance
    3. G0002:+000.000 - move along Z-axis in specified direcrion on set distance
    4. G0003:+000.000 - move along A-axis in specified direcrion on set distance
    
    ------------------------------------------------------------------------------------------------
    B. EUROMAP INTERFACE COMMANDS
    ------------------------------------------------------------------------------------------------
    1. G0004 - E000 (device op mode enable)
    2. G0005 - E001 (device op mode disable)
    3. G0006 - E002 (start handling device operation cycle) 
    4. G0007 - E003 (finish handling device operation cycle) 
    5. G0008 - E004 (set device emergency condition)
    6. G0009 - E005 (drop device emergency condition)
    7. G0010 - E006 (move ejector in forward position)
    8. G0011 - E007 (move ejector in back position)
    9. G0012 - E008 (enable moulding removal (set core pullers to position 2))
    A. G0013 - E009 (prepare next moulding cycle (set core pullers to position 1))
    B. G0014 - E010 (enabling mould closure)
    C. G0015 - E011 (disabling mould closure)
    D. G0016 - E012 (set mould area free)
    E. G0017 - E013 (set mould area occupied)

    ------------------------------------------------------------------------------------------------
    C. TECHNOLOGICAL SIGNALS COMMANDS
    ------------------------------------------------------------------------------------------------
    1. G0018 - set TS state
    2. G0019 - check sensor state
     
    ------------------------------------------------------------------------------------------------
    D. EXTERNAL MACHINERY CONTROL COMMANDS (RS-485)
    ------------------------------------------------------------------------------------------------
    1. G0020:XXXXXX@MMMM - send XXXXXX command to 0xMMMM adressed device via RS-485 

    ------------------------------------------------------------------------------------------------
    D. PROGRAMMING COMMANDS
    ------------------------------------------------------------------------------------------------
    1. G0021:XXXXXXXX@MMMMMMMM - save XXXXXXXX command at 0xMMMMMMMM EEPROM address
    2. G0022:MMMMMMMM          - start programm from 0xMMMMMMMM address   

    ------------------------------------------------------------------------------------------------
    E. SERVO SUBSYSTEM CONTROL COMMANDS
    ------------------------------------------------------------------------------------------------
    1. G0023  - X-AXIS SON ON
    2. G0024  - X-AXIS SON OFF
    3. G0025  - Y-AXIS SON ON
    4. G0026  - Y-AXIS SON OFF
    5. G0027  - Z-AXIS SON ON
    6. G0028  - Z-AXIS SON OFF
    7. G0029  - A-AXIS SON ON
    8. G0030  - A-AXIS SON OFF
    
    9. G0031  - X-AXIS EMGS OFF
    A. G0032  - X-AXIS EMGS OFF
    B. G0033  - Y-AXIS EMGS OFF
    C. G0034  - Y-AXIS EMGS OFF
    D. G0035  - Z-AXIS EMGS OFF
    E. G0036  - Z-AXIS EMGS OFF
    F. G0037  - A-AXIS EMGS OFF
    G. G0038  - A-AXIS EMGS OFF
    
    H. G0039:x  - set X-AXIS direction
    I. G0040:x  - set Y-AXIS direction
    J. G0041:x  - set Z-AXIS direction
    K. G0042:x  - set A-AXIS direction
    
    L. G0043  - move X-AXIS to home position
    M. G0044  - move Y-AXIS to home position
    N. G0045  - move Z-AXIS to home position
    o. G0046  - move A-AXIS to home position


    
*/
// Command processor initialization sequence
void TrinityCPInit(void);

// Command execution routine
void TrinityCommandExec(char* command);

// Command parser
static void TrinityGetCommandCode(char* command);
static void TrinityGetCommandArgs(char* command);

// Command utilities
static uint8_t G0000(void);
static uint8_t G0001(void);
static uint8_t G0002(void);
static uint8_t G0003(void);

static uint8_t G0004(void);
static uint8_t G0005(void);
static uint8_t G0006(void);
static uint8_t G0007(void);
static uint8_t G0008(void);
static uint8_t G0009(void);
static uint8_t G0010(void);
static uint8_t G0011(void);
static uint8_t G0012(void);
static uint8_t G0013(void);
static uint8_t G0014(void);
static uint8_t G0015(void);
static uint8_t G0016(void);
static uint8_t G0017(void);

static uint8_t G0018(void);
static uint8_t G0019(void);

static uint8_t G0020(void);
static uint8_t G0021(void);
static uint8_t G0022(void);

static uint8_t G0023(void);
static uint8_t G0024(void);
static uint8_t G0025(void);
static uint8_t G0026(void);
static uint8_t G0027(void);
static uint8_t G0028(void);
static uint8_t G0029(void);
static uint8_t G0030(void);
static uint8_t G0031(void);
static uint8_t G0032(void);
static uint8_t G0033(void);
static uint8_t G0034(void);
static uint8_t G0035(void);
static uint8_t G0036(void);
static uint8_t G0037(void);
static uint8_t G0038(void);
static uint8_t G0039(void);
static uint8_t G0040(void);
static uint8_t G0041(void);
static uint8_t G0042(void);
static uint8_t G0043(void);
static uint8_t G0044(void);
static uint8_t G0045(void);
static uint8_t G0046(void);

#endif
