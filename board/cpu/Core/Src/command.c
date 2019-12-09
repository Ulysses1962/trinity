#include "command.h"
#include "memory.h"
#include "ctype.h"

#define func_name(n) f_##n
#define func_definition(container, type, func) container.name = #func; container.proc_type = type; container.f_##type = func;

#define FUNCTION_NUM 42
#define TYPES_NUM 9

//=============================================================================
// INTERPRETER FINISHED STATE MACHINE STATES ANT PROC TYPES
//=============================================================================
typedef enum {OP_NAME, ARGS} INTERPRETER_STATE;

typedef void (*type_0)(void);
typedef void (*type_1)(double, double, double, double);
typedef void (*type_2)(double, double, double, double, double);
typedef void (*type_3)(int);
typedef void (*type_4)(int, int);
typedef void (*type_5)(int, int, int);
typedef void (*type_6)(int, int, int, int);
typedef void (*type_7)(int, char*);
typedef void (*type_8)(double, double);

typedef struct {
    char* name;
    uint8_t proc_type;
    union {
        type_0 f_0;
        type_1 f_1;
        type_2 f_2;
        type_3 f_3;
        type_4 f_4;
        type_5 f_5;
        type_6 f_6;
        type_7 f_7;
        type_8 f_8;
    };
} EXEC_INFO;

EXEC_INFO exec_tbl[FUNCTION_NUM];


UART_HandleTypeDef* emap_port = &huart1;
UART_HandleTypeDef* extm_port = &huart2;
SPI_HandleTypeDef*  eprm_port = &hspi1;
SPI_HandleTypeDef*  srvo_port = &hspi2;
SPI_HandleTypeDef*  tsgl_port = &hspi3;

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

//=============================================================================
// TRINITY SERVO CONTROLLER GLOBAL VATIABLES
//=============================================================================
static char* device_command;

static bool op_complete_X = true;
static bool op_complete_Y = true;
static bool op_complete_Z = true;
static bool op_complete_A = true;

// Current position 
static double curr_x;
static double curr_y;
static double curr_z;
static double curr_a;

static uint16_t ts_state     = 0x0000;
static uint16_t sensor_state = 0x0000;
static TRINITY_COORD_SYSTEM coord_system = ABSOLUTE;

// Flags
static bool EMAP_ACK_RECEIVED = false;
static bool DEVICE_ALARM      = false;

// SERVO SUBSYSTEM STATE. !!!IMPIORTANT!!! MUST be transmitted to subsystem in MSB order
static SERVO_CONTROL_STRUCTURE control_struct;
static TRINITY_STATE controller_state       = INIT;
static const uint8_t SERVO_SETTINGS_EXISTS  = 0xae;
static TRINITY_SETTINGS controller_settings;
                            
//-----------------------------------------------------------------------------
// TRINITY SETTINGS BASE MEMORY ADDRESSES
//-----------------------------------------------------------------------------
static const uint8_t SETTINGS_BLOCK         = 0;
static const uint8_t SETTINGS_SECTOR        = 0;
static const uint8_t SERVO_SGS_EXISTS_PAGE  = 0;
static const uint8_t SETTINGS_PAGE          = 1;

static uint8_t  PROGRAMM_BLOCK  = 1;
static uint8_t  PROGRAMM_SECTOR = 0;
static uint8_t  PROGRAMM_PAGE   = 0; 
static uint16_t const PAGE_SIZE = 256;

static uint16_t p_length     = 0;                            
static char page_buffer[PAGE_SIZE];
static bool page_complete    = false;

//-----------------------------------------------------------------------------
// TRINITY common interface routines
//-----------------------------------------------------------------------------
void TrinityCPInit(void) {
    device_command  = (char*)calloc(PGM_BUFFER_SIZE, sizeof(uint8_t));
    memset (page_buffer, 0x00, PAGE_SIZE);
    /**
     * ------------------------------------------------------------------------
     * DEFAULT SERVO CONTROL SIGNALS STATE
     * ------------------------------------------------------------------------
     * AXIS_EMGS: CLOSED (HIGH)
     * AXIS_CCLR: OPEN (LOW)
     * AXIS_SPD0: CLOSED (HIGH)
     * AXIS_DIR : OPEN (LOW)
     * AXIS_SON : OPEN (LOW)
     * AXIS_S_P : OPEN (LOW)
     * AXIS_SPD1: CLOSED (HIGH)
     * AXIS_IND : OPEN (LOW)
    */
    control_struct.A_AXIS_DIR    = FORWARD;
    control_struct.A_AXIS_EMGS   = 1;
    control_struct.A_AXIS_IND    = 0;
    control_struct.A_AXIS_S_P    = 0;
    control_struct.A_AXIS_SON    = 0;
    control_struct.A_AXIS_SPD0   = 1;
    control_struct.A_AXIS_SPD1   = 1;
    control_struct.A_AXIS_CCLR   = 0;
    
    control_struct.X_AXIS_DIR    = FORWARD;
    control_struct.X_AXIS_EMGS   = 1;
    control_struct.X_AXIS_IND    = 0;
    control_struct.X_AXIS_S_P    = 0;
    control_struct.X_AXIS_SON    = 0;
    control_struct.X_AXIS_SPD0   = 1;
    control_struct.X_AXIS_SPD1   = 1;
    control_struct.X_AXIS_CCLR   = 0;

    control_struct.Y_AXIS_DIR    = FORWARD;
    control_struct.Y_AXIS_EMGS   = 1;
    control_struct.Y_AXIS_IND    = 0;
    control_struct.Y_AXIS_S_P    = 0;
    control_struct.Y_AXIS_SON    = 0;
    control_struct.Y_AXIS_SPD0   = 1;
    control_struct.Y_AXIS_SPD1   = 1;
    control_struct.Y_AXIS_CCLR   = 0;

    control_struct.Z_AXIS_DIR    = FORWARD;
    control_struct.Z_AXIS_EMGS   = 1;
    control_struct.Z_AXIS_IND    = 0;
    control_struct.Z_AXIS_S_P    = 0;
    control_struct.Z_AXIS_SON    = 0;
    control_struct.Z_AXIS_SPD0   = 1;
    control_struct.Z_AXIS_SPD1   = 1;
    control_struct.Z_AXIS_CCLR   = 0;

    HAL_SPI_Transmit(srvo_port, (uint8_t*)&control_struct, sizeof(SERVO_CONTROL_STRUCTURE), 200);

    TrinityInitExecTbl();
    
    // LOAD PRESETS
    TrinityLoadSettings();
    if (TrinityIsPresetsValid()) {
        curr_x = controller_settings.X_REF_COORD;
        curr_y = controller_settings.Y_REF_COORD;
        curr_z = controller_settings.Z_REF_COORD;
        curr_a = controller_settings.A_REF_COORD;
    } else {
        curr_x = curr_y = curr_z = curr_a = 0;
    }

    // EUROMAP INITIAL SETTINGS

    G28(0, 0, 0, 0);
}

void TrinityProgrammExec(char* command) {
    
}



//=============================================================================
// COMMAND EXTRACTING AND PARSING ROUTINES
//=============================================================================
void TrinityInitExecTbl(void) {
    for (int t = 0; t < TYPES_NUM; ++t) {
        switch (t) {
            case 0: 
            func_definition(exec_tbl[0], 0, G90); 
            func_definition(exec_tbl[1], 0, G91); 
            func_definition(exec_tbl[2], 0, E00); 
            func_definition(exec_tbl[3], 0, E01); 
            func_definition(exec_tbl[4], 0, E02); 
            func_definition(exec_tbl[5], 0, E03); 
            func_definition(exec_tbl[6], 0, E04); 
            func_definition(exec_tbl[7], 0, E05); 
            func_definition(exec_tbl[8], 0, E06); 
            func_definition(exec_tbl[9], 0, E07); 
            func_definition(exec_tbl[10], 0, E08); 
            func_definition(exec_tbl[11], 0, E09); 
            func_definition(exec_tbl[12], 0, E10); 
            func_definition(exec_tbl[13], 0, E11); 
            func_definition(exec_tbl[14], 0, E12); 
            func_definition(exec_tbl[15], 0, E13); 
            func_definition(exec_tbl[16], 0, C01); 
            func_definition(exec_tbl[17], 0, W01); 
            break;

            case 1: 
            func_definition(exec_tbl[18], 1, G00); 
            func_definition(exec_tbl[19], 1, G20); 
            func_definition(exec_tbl[20], 1, G21); 
            func_definition(exec_tbl[21], 1, G28); 
            break;

            case 2: 
            func_definition(exec_tbl[22], 2, G01); 
            func_definition(exec_tbl[23], 2, G02); 
            func_definition(exec_tbl[24], 2, G03); 
            break;
            
            case 3: 
            func_definition(exec_tbl[25], 3, G04); 
            func_definition(exec_tbl[26], 3, W08); 
            func_definition(exec_tbl[27], 3, W09); 
            break;
            
            case 4: 
            func_definition(exec_tbl[28], 4, M06); 
            func_definition(exec_tbl[29], 4, M98); 
            func_definition(exec_tbl[30], 4, W02); 
            func_definition(exec_tbl[31], 4, W03); 
            func_definition(exec_tbl[32], 4, W04); 
            func_definition(exec_tbl[33], 4, W05); 
            break;
            
            case 5: 
            func_definition(exec_tbl[34], 5, W00); 
            break;
            
            case 6: 
            func_definition(exec_tbl[35], 6, W06); 
            func_definition(exec_tbl[36], 6, W07); 
            break;
            
            case 7: 
            func_definition(exec_tbl[37], 7, C00); 
            break;

            case 8: 
            func_definition(exec_tbl[38], 8, G00_X); 
            func_definition(exec_tbl[39], 8, G00_Y); 
            func_definition(exec_tbl[40], 8, G00_Z); 
            func_definition(exec_tbl[41], 8, G00_A); 
            break;
            
            default: break;
        }   
    }
}

void TrinityCommandInterpreter(char* command) {
    INTERPRETER_STATE state = OP_NAME;

    char sym, *name, *args[5];
    int  current_arg = 0, iter = 0;

    // Initialization sequence
    name = (char *)calloc(10, sizeof(char));
    for (int i = 0; i < 5; ++i) {
        args[i] = (char *)calloc(10, sizeof(char));
    }

    while ((sym = *command++) != '\r') {
        switch (state) {
            case OP_NAME: 
                if (sym != 0x20) *(name + iter++) = sym;     
                else {
                    iter = 0;
                    state = ARGS;
                }
            break;
            
            case ARGS:
                if (isdigit(sym) || sym == '.' || sym == '"') *(args[current_arg] + iter++) = sym;
                else if (sym == 0x20) {
                    iter = 0;
                    current_arg++;
                }
            break;
            default: break;    
        }
    }
    
    // Parsing complete! Start execution..
    iter = 0;
    bool success = true;
    while (strcmp(name, exec_tbl[iter].name)) {
        iter++;
        if (iter == FUNCTION_NUM) { success = false; break; }
    }    
    
    if (success) {
        if (exec_tbl[iter].proc_type == 0) exec_tbl[iter].func_name(0)();
        else if (exec_tbl[iter].proc_type == 1) exec_tbl[iter].func_name(1)(atof(args[0]), atof(args[1]), atof(args[2]), atof(args[3]));
        else if (exec_tbl[iter].proc_type == 2) exec_tbl[iter].func_name(2)(atof(args[0]), atof(args[1]), atof(args[2]), atof(args[3]), atof(args[4]));
        else if (exec_tbl[iter].proc_type == 3) exec_tbl[iter].func_name(3)(atoi(args[0]));
        else if (exec_tbl[iter].proc_type == 4) exec_tbl[iter].func_name(4)(atoi(args[0]), atoi(args[1]));
        else if (exec_tbl[iter].proc_type == 5) exec_tbl[iter].func_name(5)(atoi(args[0]), atoi(args[1]), atoi(args[2]));
        else if (exec_tbl[iter].proc_type == 6) exec_tbl[iter].func_name(6)(atoi(args[0]), atoi(args[1]), atoi(args[2]), atoi(args[3]));
        else if (exec_tbl[iter].proc_type == 7) exec_tbl[iter].func_name(7)(atoi(args[0]), args[1]);
        else if (exec_tbl[iter].proc_type == 8) exec_tbl[iter].func_name(8)(atof(args[0]), atof(args[1]));       
    }

    free(name);
    for (int i = 0; i < 5; ++i) free(args[i]);
}


