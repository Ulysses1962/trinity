#include "main.h"
#include "ctype.h"

#ifndef __INTERPRETER_H
#define __INTERPRETER_H

#define TEST_MODE

#define COMMAND_QUEUE_SIZE 32
#define COMMAND_LENGTH 64

/** ------------------------------------------------------------------------------------------------
    COMMAND LIST
    ------------------------------------------------------------------------------------------------
    A. SERVO CONTROL COMMANDS (IMPORTANT!!! TRYNITY USES ONLY CARTESIAN METRIC COORDINATE SYSTEM!)
    ------------------------------------------------------------------------------------------------
    1. G00      - fast feed to specified pos (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Ammm.nnn) 
    2. G01      - linear interpolation to specified pos (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Ammm.nnn Fm.n)
    3. G02      - circular interpolation CW (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Rmmm.nnn Fm.n) 
                  R - interpolation radius F - interpolation speed
    4. G03      - circular interpolation CCW. Parameters are just the same
    5. G04      - programm execution delay (params: Pmmmmm (mmmm - in milliseconds)
    6. G20      - set reference point position (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Ammm.nnn)
    7. G21      - set resolution for axis (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Ammm.nnn)
    8. G28      - return to reference point (params: Xmmm.nnn Ymmm.nnn Zmmm.nnn Ammm.nnn - these are intermediate point coordinates)
    9. G90      - use absolute coordinates
    a. G91      - use relative coordinates
    b. M30      - programm end and rewind (cycle close)

    ------------------------------------------------------------------------------------------------
    B. EUROMAP INTERFACE COMMANDS (NO PARAMETERS NEEDED)
    ------------------------------------------------------------------------------------------------
    1. E00      - (device op mode enable)
    2. E01      - (device op mode disable)
    3. E02      - (start handling device operation cycle) 
    4. E03      - (finish handling device operation cycle) 
    5. E04      - (set device emergency condition)
    6. E05      - (drop device emergency condition)
    7. E06      - (move ejector in forward position)
    8. E07      - (move ejector in back position)
    9. E08      - (enable moulding removal (set core pullers to position 2))
    A. E09      - (prepare next moulding cycle (set core pullers to position 1))
    B. E10      - (enabling mould closure)
    C. E11      - (disabling mould closure)
    D. E12      - (set mould area free)
    E. E13      - (set mould area occupied)

    ------------------------------------------------------------------------------------------------
    C. TECHNOLOGICAL SIGNALS COMMANDS
    ------------------------------------------------------------------------------------------------
    1. M06      - set TS state (params: Tnn Sm, where nn - signal line number, m - state. 0 - signal inactive, 1 - active)

    ------------------------------------------------------------------------------------------------
    D. EXTERNAL MACHINERY CONTROL COMMANDS (RS-485)
    ------------------------------------------------------------------------------------------------
    1. C00      - send external machinery control command via RS-485 (params: ADDRxxxx CMDyyyyyy) 
    2. C01      - get external machinery responce (response is written to default RS-485 buffer)

    ------------------------------------------------------------------------------------------------
    D. PROGRAMMING COMMANDS
    ------------------------------------------------------------------------------------------------
    1. W00      - start programming (params: Bnnn Smm Pxx, where nnn, mm and xx are starting block, sector and page)
                  all data, received after that considered to be programm codes and has to be written to FLASH memory           
    2. W01      - stop programming procedure
    3. M98      - programm start (params: Onnnmmxx Nxxxxxx, where nnnmmxx - is programm code, constructed from
                  block, sector and page number, xxxxxx - is the number of cycles (infinite cysle when 000000)) 
    4. w02-W05  - set axis bindings (params: Xn | Yn |Zn | An Tmm, where n = 0, 1, 2 (0 means we set binding for MIN position
                  for axis, 1 - we bind MAX position), mm is sensor line which is binded)     
    5. W06      - set axis activity (params: Xn Yn Zn An, where n - axis activity (1 - active, 0 - inactive)) 
    6. W07      - set emergency stop condition for axis (params are just the same)  
    7. W08      - wait for sensor line become active (params: SENxx, xx - sensor line number)   
    8. W09      - wait until specified line become inactive. Params are just the same

*/
//-----------------------------------------------------------------------------
// EXTERN DATA
//-----------------------------------------------------------------------------
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;

extern uint16_t SERVO_X_CTL_PRESCALER;
extern uint16_t SERVO_Y_CTL_PRESCALER;
extern uint16_t SERVO_Z_CTL_PRESCALER;
extern uint16_t SERVO_A_CTL_PRESCALER;

//-----------------------------------------------------------------------------
// GENERAL DATA TYPES
//-----------------------------------------------------------------------------
typedef enum {FORWARD, BACKWARD} TRINITY_SERVO_DIRECTION;
typedef enum {CH_X, CH_Y, CH_Z, CH_A} TRINITY_AXIS;
typedef enum {PROGRAMMING, RUN, INIT} TRINITY_STATE;
typedef enum {ABSOLUTE, RELATIVE} TRINITY_COORD_SYSTEM;

//-----------------------------------------------------------------------------
// CONTROL STRUCTURES
//-----------------------------------------------------------------------------
// SERVO control signals (for use with DELTA ASDA-B2 sero controller)
typedef struct {
    uint32_t A_AXIS_EMGS: 1;
    uint32_t A_AXIS_CCLR: 1;
    uint32_t A_AXIS_SPD0: 1;
    TRINITY_SERVO_DIRECTION A_AXIS_DIR : 1;
    uint32_t A_AXIS_SON : 1;
    uint32_t A_AXIS_S_P : 1;
    uint32_t A_AXIS_SPD1: 1;
    uint32_t A_AXIS_IND : 1;

    uint32_t Z_AXIS_EMGS: 1;
    uint32_t Z_AXIS_CCLR: 1;
    uint32_t Z_AXIS_SPD0: 1;
    TRINITY_SERVO_DIRECTION Z_AXIS_DIR : 1;
    uint32_t Z_AXIS_SON : 1;
    uint32_t Z_AXIS_S_P : 1;
    uint32_t Z_AXIS_SPD1: 1;
    uint32_t Z_AXIS_IND : 1;

    uint32_t Y_AXIS_EMGS: 1;
    uint32_t Y_AXIS_CCLR: 1;
    uint32_t Y_AXIS_SPD0: 1;
    TRINITY_SERVO_DIRECTION Y_AXIS_DIR : 1;
    uint32_t Y_AXIS_SON : 1;
    uint32_t Y_AXIS_S_P : 1;
    uint32_t Y_AXIS_SPD1: 1;
    uint32_t Y_AXIS_IND : 1;

    uint32_t X_AXIS_EMGS: 1;
    uint32_t X_AXIS_CCLR: 1;
    uint32_t X_AXIS_SPD0: 1;
    TRINITY_SERVO_DIRECTION X_AXIS_DIR : 1;
    uint32_t X_AXIS_SON : 1;
    uint32_t X_AXIS_S_P : 1;
    uint32_t X_AXIS_SPD1: 1;
    uint32_t X_AXIS_IND : 1;
} SERVO_CONTROL_STRUCTURE;  

// TRINITY SETTINGS STRUCTURE
typedef struct {
    uint8_t  preset_valid;
    double   X_AXIS_RESOLUTION;
    double   Y_AXIS_RESOLUTION;
    double   Z_AXIS_RESOLUTION;
    double   A_AXIS_RESOLUTION;

    double   X_MAX_FEED;
    double   Y_MAX_FEED;
    double   Z_MAX_FEED;
    double   A_MAX_FEED;

    uint16_t X_MAX_POS_SENSOR_MASK;
    uint16_t X_MIN_POS_SENSOR_MASK;
    uint16_t Y_MAX_POS_SENSOR_MASK;
    uint16_t Y_MIN_POS_SENSOR_MASK;
    uint16_t Z_MAX_POS_SENSOR_MASK;
    uint16_t Z_MIN_POS_SENSOR_MASK;
    uint16_t A_MAX_POS_SENSOR_MASK;
    uint16_t A_MIN_POS_SENSOR_MASK;

    double X_REF_COORD;  
    double Y_REF_COORD;      
    double Z_REF_COORD;    
    double A_REF_COORD;   

    TRINITY_COORD_SYSTEM C_SYSTEM;      
} TRINITY_SETTINGS;

//=============================================================================
// INTERPRETER FINISHED STATE MACHINE STATES ANT PROC TYPES
//=============================================================================
typedef enum {OP_NAME, DELIMITER, ARGS} PARSER_STATE;

//=============================================================================
// TRINITY INTERPRETER STRUCTURE
//=============================================================================
typedef void (*utility_proc)(char**);

typedef struct {
    char* name; 
    utility_proc func;
} EXEC_INFO;

typedef struct sCommandQueue {
    char* queue[COMMAND_QUEUE_SIZE];
    void  (*TrinityPUSHCmd)(struct sCommandQueue*, char*);
    char* (*TrinityPOPCmd)(struct sCommandQueue*);
    int queue_tail;
    int queue_head;
} TRINITY_COMMAND_QUEUE;

typedef struct sInterpreter {
    EXEC_INFO* exec_tbl;
    TRINITY_COMMAND_QUEUE* queue;
    bool (*peek)(struct sInterpreter*);
    bool (*exec)(struct sInterpreter*);
    uint32_t iterator;
} TRINITY_INTERPRETER;

TRINITY_COMMAND_QUEUE* new_cmd_queue(void);
TRINITY_INTERPRETER* new_interpreter(void);

// Init routine
void TrinityInterpreterInit(void);
void TrinityRun (char **args);

#endif
