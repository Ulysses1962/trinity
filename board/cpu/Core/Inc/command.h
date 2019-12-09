#include "main.h"

#ifndef __COMMAND_H__
#define __COMMAND_H__

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

//-----------------------------------------------------------------------------
// GENERAL DATA TYPES
//-----------------------------------------------------------------------------
typedef enum {FORWARD, BACKWARD} TRINITY_SERVO_DIRECTION;
typedef enum {CH_X, CH_Y, CH_Z, CH_A} TRINITY_AXIS;
typedef enum {INIT, PROGRAMMING, RUN} TRINITY_STATE;
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
// Command processor initialization sequence
void TrinityCPInit(void);
// Command execution routine
void TrinityProgrammExec(char* command);
// Command parser
static void TrinityCommandInterpreter(char* command);

//=============================================================================
// Utility routines
//=============================================================================
static uint32_t TrinitySetAxisDirection(TRINITY_AXIS axis, TRINITY_SERVO_DIRECTION dir);
static uint32_t TrinitySetAxisSpeed(TRINITY_AXIS axis, double feed);
static uint32_t TrinitySaveSettings(void);
static uint32_t TrinityLoadSettings(void);
static bool TrinityIsPresetsValid(void);
static uint32_t TrinityMoveToZero(void);
static void TrinityDeviceAlarm(bool alarm);
static bool TrinityGetSensorState(uint8_t sens_line);
static void TrinityInitExecTbl(void);

//=============================================================================
// Command routines
//=============================================================================
static void G00 (double x, double y, double z, double a);
static void G00_X (double x, double feed); 
static void G00_Y (double y, double feed); 
static void G00_Z (double z, double feed); 
static void G00_A (double a, double feed); 
static void G01 (double x, double y, double z, double a, double feed);
static void G02 (double x, double y, double z, double r, double feed); 
static void G03 (double x, double y, double z, double r, double feed);
static void G04 (uint32_t m_delay);
static void G20 (double x, double y, double z, double a);
static void G21 (double res_x, double res_y, double res_z, double res_a);
static void G28 (double x, double y, double z, double a);
static void G90 (void);
static void G91 (void);

static void E00 (void);
static void E01 (void);
static void E02 (void); 
static void E03 (void); 
static void E04 (void);
static void E05 (void);
static void E06 (void);
static void E07 (void);
static void E08 (void);
static void E09 (void);
static void E10 (void);
static void E11 (void);
static void E12 (void);
static void E13 (void);
static bool TrinityGetEMAPAck(void);

static void M06 (uint16_t line, uint8_t state);

static void C00 (uint16_t addr, char* command);
static void C01 (void);

static void W00 (uint16_t start_block, uint16_t start_sector, uint16_t start_page);
static void W01 (void);
static void M98 (uint32_t start_addr, uint32_t num_cycles);
static void W02 (uint8_t selector, uint16_t sens_line);
static void W03 (uint8_t selector, uint16_t sens_line);
static void W04 (uint8_t selector, uint16_t sens_line);
static void W05 (uint8_t selector, uint16_t sens_line);
static void W06 (uint8_t x_active, uint8_t y_active, uint8_t z_active, uint8_t a_active);
static void W07 (uint8_t x_emgs, uint8_t y_emgs, uint8_t z_emgs, uint8_t a_emgs);
static void W08 (uint16_t sens_line);
static void W09 (uint16_t sens_line);

//-----------------------------------------------------------------------------
// INTERRUPT SERVICE CALLBACKS
//-----------------------------------------------------------------------------
void onEmapCommandReceived(void);

#endif
