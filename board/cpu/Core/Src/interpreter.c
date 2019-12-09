#include "interpreter.h"
#include "memory.h"

#define func_desc(func) {#func, func}
#define FUNCTION_NUM 42

UART_HandleTypeDef* emap_port = &huart1;
UART_HandleTypeDef* extm_port = &huart2;
SPI_HandleTypeDef*  eprm_port = &hspi1;
SPI_HandleTypeDef*  srvo_port = &hspi2;
SPI_HandleTypeDef*  tsgl_port = &hspi3;

//=============================================================================
// TRINITY INTERPRETER GLOBALS
//=============================================================================
// Operation complete flags
static bool op_complete_X = true;
static bool op_complete_Y = true;
static bool op_complete_Z = true;
static bool op_complete_A = true;

// Servo axis ready flags
static bool servo_x_ready = false;
static bool servo_y_ready = false;
static bool servo_z_ready = false;
static bool servo_a_ready = false;

// Current position 
static double curr_x;
static double curr_y;
static double curr_z;
static double curr_a;

// Signals, sensors and coordinate system
static uint16_t ts_state     = 0x0000;
static uint16_t sensor_state = 0x0000;
static TRINITY_COORD_SYSTEM coord_system = ABSOLUTE;

// EUROMAP flags
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
static const uint8_t SETTINGS_PAGE          = 0;

static const uint8_t STARTUP_BLOCK          = 0;
static const uint8_t STARTUP_SECTOR         = 0;
static const uint8_t STARTUP_PAGE           = 1;

static uint8_t  PROGRAMM_BLOCK  = 1;
static uint8_t  PROGRAMM_SECTOR = 0;
static uint8_t  PROGRAMM_PAGE   = 0; 
static uint16_t const PAGE_SIZE = 256;
static uint32_t PROGRAM_CYCLES;

static uint16_t p_length     = 0;                            
static char page_buffer[PAGE_SIZE];
static bool page_complete    = false;

//=============================================================================
// INTERPRETER OBJECT
//=============================================================================
 TRINITY_INTERPRETER* itp = NULL; 

//=============================================================================
// TEST DATA
//=============================================================================
char* test_pgm = "G90\r\nG00 X0.78 Y8.9 Z34.8 A0.00\r\nG28 X0.78 Y8.9 Z34.8 A0.00\r\nE00\r\nE05\r\nG01  X0.78 Y8.9 Z34.8 A0.00 F0.02\r\n";

//=============================================================================
// UTILITY ROUTINES DEFINITION
//=============================================================================
static uint32_t TrinitySaveSettings(void);
static uint32_t TrinityLoadSettings(void);
static bool TrinityIsPresetsValid(void);

// TYPE_0 routines
static void G90 (char **args);
static void G91 (char **args); 
static void E00 (char **args); 
static void E01 (char **args); 
static void E02 (char **args); 
static void E03 (char **args);  
static void E04 (char **args); 
static void E05 (char **args); 
static void E06 (char **args); 
static void E07 (char **args); 
static void E08 (char **args); 
static void E09 (char **args); 
static void E10 (char **args); 
static void E11 (char **args); 
static void E12 (char **args); 
static void E13 (char **args); 
static void C01 (char **args); 
static void W01 (char **args);
static void M30 (char **args);

// TYPE_1 routines
static void G00 (char **args); 
static void G20 (char **args); 
static void G21 (char **args); 
static void G28 (char **args); 

// TYPE_2 routines11
static void G01 (char **args);
static void G02 (char **args); 
static void G03 (char **args);

// TYPE_3 routines
static void G04 (char **args);
static void W08 (char **args);
static void W09 (char **args);

// TYPE_4 routines
static void M06 (char **args);
static void W02 (char **args);
static void W03 (char **args);
static void W04 (char **args);
static void W05 (char **args);

// TYPE_5 routines
static void W00 (char **args);

// TYPE_6 routines
static void W06 (char **args);
static void W07 (char **args);

// TYPE_7 routines
static void C00 (char **args);

// TYPE_8 routines
static void G00_X (char **args);
static void G00_Y (char **args);
static void G00_Z (char **args);
static void G00_A (char **args);

//=============================================================================
// INTERPRETER EXEC TABLE
//=============================================================================
static EXEC_INFO exec_tbl[FUNCTION_NUM] = {
    func_desc(G90), func_desc(G91), func_desc(E00), func_desc(E01), func_desc(E02), func_desc(E03), func_desc(E04), func_desc(E05), func_desc(E06), func_desc(E07), 
    func_desc(E08), func_desc(E09), func_desc(E10), func_desc(E11), func_desc(E12), func_desc(E13), func_desc(C01), func_desc(W01), func_desc(M30), func_desc(G00), 
    func_desc(G20), func_desc(G21), func_desc(G28), func_desc(G01), func_desc(G02), func_desc(G03), func_desc(G04), func_desc(W08), func_desc(W09), func_desc(M06), 
    func_desc(W02), func_desc(W03), func_desc(W04), func_desc(W05), func_desc(W00), func_desc(W06), func_desc(W07), func_desc(C00), func_desc(G00_X), func_desc(G00_Y), 
    func_desc(G00_Z), func_desc(G00_A)};


//-----------------------------------------------------------------------------
// INTERPRETER STRUCT ROUTINES
//-----------------------------------------------------------------------------
static void TrinityMoveToZero(void) {
    if (!TrinityIsPresetsValid()) return;

    char args[2][20];
    uint32_t address = 1;
    uint32_t cycles  = 1;
    sprintf(args[0], "%u", address);
    sprintf(args[1], "%u", cycles);
    
    // Run initialization sequense
    TrinityRun((char**)args);

}

void TrinityInterpreterInit(void) {
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

    // LOAD PRESETS
    TrinityLoadSettings();

    // MOVE TO ZERO POINT
    TrinityMoveToZero();

    if (TrinityIsPresetsValid()) {
        curr_x = controller_settings.X_REF_COORD;
        curr_y = controller_settings.Y_REF_COORD;
        curr_z = controller_settings.Z_REF_COORD;
        curr_a = controller_settings.A_REF_COORD;
    } else {
        curr_x = curr_y = curr_z = curr_a = 0;
    }

    itp = new_interpreter();
}

// AUX routines..
static void command_push(TRINITY_COMMAND_QUEUE* cmd_queue, char* command) {
    if (cmd_queue == NULL) return;
    
    memset(cmd_queue->queue[cmd_queue->queue_tail], 0x00, COMMAND_LENGTH);
    memcpy(cmd_queue->queue[cmd_queue->queue_tail], command, strlen(command));
    cmd_queue->queue_tail = (cmd_queue->queue_tail + 1) % COMMAND_QUEUE_SIZE;
}

static char* command_pop(TRINITY_COMMAND_QUEUE* cmd_queue) {
    if (cmd_queue == NULL) return NULL;

    char* cmd = NULL;
    if (cmd_queue->queue_head != cmd_queue->queue_tail) {
        cmd = cmd_queue->queue[cmd_queue->queue_head];
        cmd_queue->queue_head = (cmd_queue->queue_head + 1) % COMMAND_QUEUE_SIZE;
    } 
    return cmd;
}

static bool command_exec(TRINITY_INTERPRETER* interpreter) {
    char* command = interpreter->queue->TrinityPOPCmd(interpreter->queue);
    if (command == NULL) return false;

    PARSER_STATE state = OP_NAME;
    char sym, name[10], args[5][10];
    int  current_arg = 0, iter = 0, command_iterator = 0;

    // Command parsing sequence
    while ((sym = *(command + command_iterator++)) != '\r') {
        switch (state) {
            case OP_NAME: 
                if (sym != 0x20) *(name + iter++) = sym;     
                else {
                    iter = 0;
                    state = DELIMITER;
                }
            break;

            case DELIMITER:
                if (sym != 0x20) state = ARGS;
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
    
    // Parsing complete! Search execution table for appropriate routine
    iter = 0;
    bool success = true;
    while (strcmp(name, interpreter->exec_tbl[iter++].name)) {
        if (iter == FUNCTION_NUM) { success = false; break; }
    }    
    
    // If routine found, exec it..
    if (success) interpreter->exec_tbl[--iter].func(args);

    return true;
}   

static bool command_test_extract(TRINITY_INTERPRETER* interpreter) {
    if (*(test_pgm + interpreter->iterator) == 0x00) return false;
    
    char cmd[COMMAND_LENGTH], sym;
    int command_iterator = 0;
    memset(cmd, 0x00, COMMAND_LENGTH);
       
    while ((sym = *(test_pgm + interpreter->iterator++)) != '\n') cmd[command_iterator++] = sym;
    interpreter->queue->TrinityPUSHCmd(interpreter->queue, cmd);
    return true;
}

static bool command_extract(TRINITY_INTERPRETER* interpreter) {
    if (*(page_buffer + interpreter->iterator) == 0x00) return false;
    
    char cmd[COMMAND_LENGTH], sym;
    int command_iterator = 0;
    memset(cmd, 0x00, COMMAND_LENGTH);
       
    while ((sym = *(page_buffer + interpreter->iterator++)) != '\n') cmd[command_iterator++] = sym;
    interpreter->queue->TrinityPUSHCmd(interpreter->queue, cmd);
    
    return true;
}

// Constructors
TRINITY_COMMAND_QUEUE* new_cmd_queue(void) {
    TRINITY_COMMAND_QUEUE* cmd_queue = (TRINITY_COMMAND_QUEUE*)calloc(1, sizeof(TRINITY_COMMAND_QUEUE));

    if (cmd_queue) {
        cmd_queue->queue_head = 0;
        cmd_queue->queue_tail = 0;
        cmd_queue->TrinityPOPCmd  = command_pop;
        cmd_queue->TrinityPUSHCmd = command_push;

        for (int i = 0; i < COMMAND_QUEUE_SIZE; ++i) cmd_queue->queue[i] = (char*)calloc(COMMAND_LENGTH, sizeof(char));
    }

    return cmd_queue;
}

TRINITY_INTERPRETER* new_interpreter(void) {
    TRINITY_INTERPRETER* interpreter = (TRINITY_INTERPRETER*)calloc(1, sizeof(TRINITY_INTERPRETER));

    if (interpreter) {
        interpreter->queue = new_cmd_queue();
        interpreter->exec_tbl = exec_tbl;
        interpreter->exec = command_exec;
#ifdef TEST_MODE        
        interpreter->peek = command_test_extract;
#else
        interpreter->peek = command_extract;
#endif
        interpreter->iterator = 0;
    }

    return interpreter;
}

#ifdef TEST_MODE
//-----------------------------------------------------------------------------
// Main entry point for test mode
//-----------------------------------------------------------------------------
int main(void) {
    TRINITY_INTERPRETER* itp = new_interpreter();

    itp->peek(itp);
    while (itp->exec(itp)) itp->peek(itp);
    printf ("PROGRAMM COMPLETE!\n");
}
//-----------------------------------------------------------------------------
#endif

//=============================================================================
// UTILITY ROOTINES
//=============================================================================
static void TrinityNextPage(void) {
    PROGRAMM_PAGE++;
    if (PROGRAMM_PAGE < 15) return;

    PROGRAMM_PAGE = 0;
    PROGRAMM_SECTOR++;
    if (PROGRAMM_SECTOR < 15) return;

    PROGRAMM_SECTOR = 0;
    PROGRAMM_BLOCK++;
}

static bool TrinityGetSensorState(uint8_t sens_line) {
    GPIO_PinState sens_state;
    uint8_t mask = 0b00000001;

    HAL_GPIO_WritePin(SENS_ADDR0_GPIO_Port, SENS_ADDR0_Pin, GPIO_PIN_SET & (sens_line & mask));
    HAL_GPIO_WritePin(SENS_ADDR1_GPIO_Port, SENS_ADDR1_Pin, GPIO_PIN_SET & (sens_line & mask << 1));
    HAL_GPIO_WritePin(SENS_ADDR2_GPIO_Port, SENS_ADDR2_Pin, GPIO_PIN_SET & (sens_line & mask << 2));
    
    HAL_GPIO_WritePin(SENS_STROBE_GPIO_Port, SENS_STROBE_Pin, GPIO_PIN_RESET);
    if (sens_line < 8) sens_state = HAL_GPIO_ReadPin(SENS_IN_A_GPIO_Port, SENS_IN_A_Pin);
    else sens_state = HAL_GPIO_ReadPin(SENS_IN_B_GPIO_Port, SENS_IN_B_Pin);
    HAL_GPIO_WritePin(SENS_STROBE_GPIO_Port, SENS_STROBE_Pin, GPIO_PIN_SET);

    return (sens_state == GPIO_PIN_SET);
}

uint32_t TrinitySaveSettings(void) {
    return TrinityWritePage(SETTINGS_BLOCK, SETTINGS_SECTOR, SETTINGS_PAGE, (uint8_t*)&controller_settings, sizeof(TRINITY_SETTINGS));
}

uint32_t TrinityLoadSettings(void) {
    return TrinityReadPage(SETTINGS_BLOCK, SETTINGS_SECTOR, SETTINGS_PAGE, (uint8_t*)&controller_settings, sizeof(TRINITY_SETTINGS));
}

bool TrinityIsPresetsValid(void) {
    return controller_settings.preset_valid == SERVO_SETTINGS_EXISTS;
}

static void TrinityDeviceAlarm(bool alarm) {
    DEVICE_ALARM = alarm;
    // to do.. Alarm indication
}

uint32_t TrinitySetAxisDirection(TRINITY_AXIS axis, TRINITY_SERVO_DIRECTION dir) {
    HAL_StatusTypeDef res = HAL_OK;

    switch (axis) {
        case CH_A: control_struct.A_AXIS_DIR = dir; break;
        case CH_Z: control_struct.Z_AXIS_DIR = dir; break;
        case CH_Y: control_struct.Y_AXIS_DIR = dir; break;
        case CH_X: control_struct.X_AXIS_DIR = dir; break;
    }
    res = HAL_SPI_Transmit(srvo_port, (uint8_t*)&control_struct, sizeof(SERVO_CONTROL_STRUCTURE), 200);

    return (uint32_t)res;
}

/**
 *  @param axis  - axis to operate 
 *  @param speed - axis speed 1..100
 */
uint32_t TrinitySetAxisSpeed(TRINITY_AXIS axis, double feed) {
    HAL_StatusTypeDef res = HAL_OK;
    uint16_t speed;

    switch (axis) {
        case CH_X: 
            HAL_TIM_Base_Stop_IT(&htim8);
            HAL_TIM_Base_Stop_IT(&htim5);
            
            speed = (uint16_t)ceil(feed / (controller_settings.X_AXIS_RESOLUTION * 60 * SERVO_PULSE_FREQ));
            if (speed < 1 || speed > 100) return (uint32_t)HAL_ERROR;
            
            TIM8->ARR  = (uint16_t)(SERVO_REFERENCE_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            TIM5->PSC  = (uint16_t)(SYS_REFERENCE_APB1_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            break;
        case CH_Y:
            HAL_TIM_Base_Stop_IT(&htim1);
            HAL_TIM_Base_Stop_IT(&htim4);

            speed = (uint16_t)ceil(feed / (controller_settings.Y_AXIS_RESOLUTION * 60 * SERVO_PULSE_FREQ));
            if (speed < 1 || speed > 100) return (uint32_t)HAL_ERROR;

            TIM1->ARR  = (uint16_t)(SERVO_REFERENCE_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            TIM4->PSC  = (uint16_t)(SYS_REFERENCE_APB1_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            break;
        case CH_Z: 
            HAL_TIM_Base_Stop_IT(&htim2);
            HAL_TIM_Base_Stop_IT(&htim3);

            speed = (uint16_t)ceil(feed / (controller_settings.Z_AXIS_RESOLUTION * 60 * SERVO_PULSE_FREQ));
            if (speed < 1 || speed > 100) return (uint32_t)HAL_ERROR;

            TIM2->ARR  = (uint16_t)(SERVO_REFERENCE_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            TIM3->PSC  = (uint16_t)(SYS_REFERENCE_APB1_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            break;
        case CH_A: 
            HAL_TIM_Base_Stop_IT(&htim9);
            HAL_TIM_Base_Stop_IT(&htim10);

            speed = (uint16_t)ceil(feed / (controller_settings.A_AXIS_RESOLUTION * 60 * SERVO_PULSE_FREQ));
            if (speed < 1 || speed > 100) return (uint32_t)HAL_ERROR;

            TIM9->ARR  = (uint16_t)(SERVO_REFERENCE_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            TIM10->PSC = (uint16_t)(SYS_REFERENCE_APB2_FREQ / SERVO_PULSE_FREQ * 0.1 * speed - 1);
            break;
    }
    HAL_Delay(100);

    return (uint32_t)res;
}

//=============================================================================
// SERVO PULSE TIMERS interrupt serving callback
//=============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    HAL_TIM_Base_Stop_IT(htim);
    
    if (htim->Instance == TIM5) {
    // X-AXIS processing    
        op_complete_X = true;
    } else if (htim->Instance == TIM4) {
    // Y-AXIS processing
        op_complete_Y = true;
    } else if (htim->Instance == TIM3) {
    // Z-AXIS processing
        op_complete_Z = true;
    } else if (htim->Instance == TIM10) {
    // A-AXIS processing
        op_complete_A = true;
    }        
}

//=============================================================================
// SERVO COMMAND UTILITIES
//=============================================================================
/**
 *  @brief  Fast feed to specified position
 *  @param x    - x coordinate of destination point
 *  @param y    - y coordinate of destination point
 *  @param z    - z coordinate of destination point
 *  @param a    - a coordinate of destination point 
 */
void G00 (char** args) {
    if (!DEVICE_ALARM) return;
    char args_m[2][10];

    memcpy(args_m[0], args[0], 10);
    sprintf(args_m[1], "%8.3f\0", controller_settings.X_MAX_FEED); 
    G00_X(args_m);

    memcpy(args_m[0], args[1], 10);
    sprintf(args_m[1], "%8.3f\0", controller_settings.Y_MAX_FEED); 
    G00_Y(args_m);

    memcpy(args_m[0], args[2], 10);
    sprintf(args_m[1], "%8.3f\0", controller_settings.Z_MAX_FEED); 
    G00_Z(args_m);

    memcpy(args_m[0], args[3], 10);
    sprintf(args_m[1], "%8.3f\0", controller_settings.A_MAX_FEED); 
    G00_A(args_m);
}

/** 
 * @brief Moves along X-axis to specified position with specified feed in mm/min
 * @param x    - X-coord of specified point
 * @param feed - movement feed in mm/min
 */
void G00_X (char** args) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dx;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_X, atof(args[1]));
    double x = atof(args[0]); 
    if (coord_system == ABSOLUTE) {
        dx = curr_x - x;
        curr_x = x;    
    } else dx = x;
    
    dir = (dx > 0) ? FORWARD : BACKWARD;
    n_pulse = (uint16_t)fabs(dx / controller_settings.X_AXIS_RESOLUTION);
    TrinitySetAxisDirection(CH_X, dir);

    TIM5->ARR  = n_pulse;
    op_complete_X = false;
    HAL_TIM_Base_Start_IT(&htim5);
}

/** 
 * @brief Moves along Y-axis to specified position with specified feed in mm/min
 * @param y    - Y-coord of specified point
 * @param feed - movement feed in mm/min
 */
void G00_Y (char** args) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dy;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_Y, atof(args[1]));
    double y = atof(args[0]);
    if (coord_system == ABSOLUTE) {
        dy = curr_y - y;
        curr_y = y;
    } else dy = y;

    dir = (dy > 0) ? FORWARD : BACKWARD;
    n_pulse = (uint16_t)fabs(dy / controller_settings.Y_AXIS_RESOLUTION);
    TrinitySetAxisDirection(CH_Y, dir);

    TIM4->ARR  = n_pulse;
    op_complete_X = false;
    HAL_TIM_Base_Start_IT(&htim4);
} 

/** 
 * @brief Moves along Z-axis to specified position with specified feed in mm/min
 * @param z    - Z-coord of specified point
 * @param feed - movement feed in mm/min
 */
void G00_Z (char** args) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dz;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_Z, atof(args[1]));
    double z = atof(args[0]);
    if ( coord_system == ABSOLUTE) {
        dz = curr_z - z;
        curr_z = z;
    } else dz = z;

    dir = (dz > 0) ? FORWARD : BACKWARD;
    n_pulse = (uint16_t)fabs(dz / controller_settings.Z_AXIS_RESOLUTION);
    TrinitySetAxisDirection(CH_Z, dir);

    TIM3->ARR  = n_pulse;
    op_complete_X = false;
    HAL_TIM_Base_Start_IT(&htim3);
} 

/** 
 * @brief Moves along A-axis to specified position with specified feed in mm/min
 * @param a    - A-coord of specified point
 * @param feed - movement feed in mm/min
 */
void G00_A (char** args) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double da;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_A, atof(args[1]));
    double a = atof(args[0]);
    if (coord_system == ABSOLUTE) {
        da = curr_a - a;
        curr_a = a;
    } else da = a;

    dir = (da > 0) ? FORWARD : BACKWARD;
    n_pulse = (uint16_t)fabs(da / controller_settings.A_AXIS_RESOLUTION);
    TrinitySetAxisDirection(CH_A, dir);

    TIM10->ARR  = n_pulse;
    op_complete_X = false;
    HAL_TIM_Base_Start_IT(&htim10);
} 

/**
 *  @brief  Linear interpolation to specified position with specified speed
 *  @param x    - x coordinate of destination point
 *  @param y    - y coordinate of destination point
 *  @param z    - z coordinate of destination point
 *  @param a    - a coordinate of destination point 
 *  @param feed - feed speed mm/min
 */
void G01 (char** args) {
    if (!DEVICE_ALARM) return;
    char args_m[2][10];

    memcpy(args_m[0], args[0], 10);
    memcpy(args_m[1], args[4], 10);
    G00_X(args_m);

    memcpy(args_m[0], args[1], 10);
    G00_Y(args_m);

    memcpy(args_m[0], args[2], 10);
    G00_Z(args_m);

    memcpy(args_m[0], args[3], 10);
    G00_A(args_m);
}

/**
 *  @brief  Circular CW interpolation to specified point using specified radius and speed
 *  @param x    - x coordinate of destination point
 *  @param y    - y coordinate of destination point
 *  @param z    - z coordinate of destination point
 *  @param r    - interpolation radius 
 *  @param feed - feed speed mm/min
 *  Reserved for future use
 */
void G02 (char** args) {}

/**
 *  @brief  Circular CCW interpolation to specified point using specified radius and speed
 *  @param x    - x coordinate of destination point
 *  @param y    - y coordinate of destination point
 *  @param z    - z coordinate of destination point
 *  @param r    - interpolation radius 
 *  @param feed - feed speed mm/min
 *  Reserved for future use
 */
void G03 (char** args) {}

/**
 *  @brief  Programm delay for specified duration
 *  @param m_delay - delay in milliseconds
 */
void G04 (char** args) {
    while (!(op_complete_A && op_complete_X && op_complete_Y && op_complete_Z));
    HAL_Delay(atoi(args[0]));
}

/**
 *  @brief  Set reference point
 *  @param x    - x-coordinate of reference point
 *  @param y    - y-coordinate of reference point
 *  @param z    - z-coordinate of reference point
 *  @param a    - a-coordinate of reference point
 */
void G20 (char** args) {
    TrinityLoadSettings();

    controller_settings.X_REF_COORD = atof(args[0]);
    controller_settings.Y_REF_COORD = atof(args[1]);
    controller_settings.Z_REF_COORD = atof(args[2]);
    controller_settings.A_REF_COORD = atof(args[3]);

    TrinitySaveSettings();
}

/**
 *  @brief  Set axis resolutions
 *  @param res_x    - x axis resolution
 *  @param res_y    - y axis resolution
 *  @param res_z    - z axis resolution
 *  @param res_a    - a axis resolution
 */
void G21 (char** args) {
    TrinityLoadSettings();

    controller_settings.X_AXIS_RESOLUTION = atof(args[0]);
    controller_settings.Y_AXIS_RESOLUTION = atof(args[1]);
    controller_settings.Z_AXIS_RESOLUTION = atof(args[2]);
    controller_settings.A_AXIS_RESOLUTION = atof(args[3]);

    TrinitySaveSettings(); 
}

/**
 *  @brief Return to reference point via the intermediate point
 *  @param x    - x axis intermediate coordinate
 *  @param y    - y axis intermediate coordinate
 *  @param z    - z axis intermediate coordinate
 *  @param a    - a axis intermediate coordinate
 */
void G28 (char** args) {
    G00(args);
    sprintf(args[0], "%8.3f\0", controller_settings.X_REF_COORD);
    sprintf(args[1], "%8.3f\0", controller_settings.Y_REF_COORD);
    sprintf(args[2], "%8.3f\0", controller_settings.Z_REF_COORD);
    sprintf(args[3], "%8.3f\0", controller_settings.A_REF_COORD);
    G00(args);
}

/**
 * @brief Use absolute coordinates
 */
void G90 (char** args) {
    coord_system = ABSOLUTE;
}

/**
 * @brief Use relative coordinates
 */
void G91 (char** args) {
    coord_system = RELATIVE;
}

static bool TrinityGetEMAPAck(void) {
   uint8_t resp[10];
   HAL_UART_Receive_IT(&huart1, resp, 10);

   while (!EMAP_ACK_RECEIVED);

   if (strstr(resp, "OK")) return true;
   else return false;
}

/**
 * @brief Device operation mode disable
 */
void E00 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E000");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Device operation mode disable
 */
void E01 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E001");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Start handling device operation cycle
 */
void E02 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E002");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Finish handling device operation cycle
 */
void E03 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E003");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set device emergency condition
 */
void E04 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E004");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    TrinityDeviceAlarm(true);
}

/**
 * @brief Drop device emergency condition
 */
void E05 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E005");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Move ejector to forward position
 */
void E06 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E006");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Move ejector to back position
 */
void E07 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E007");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set core pullers to position 2
 */
void E08 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E008");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set core pullers to position 1
 */
void E09 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E009");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Mould closure enable
 */
void E10 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E010");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Mould closure disable
 */
void E11 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E011");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set mould area free
 */
void E12 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E012");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set mould area occupied
 */
void E13 (char** args) {
    char device_cmd[PGM_BUFFER_SIZE];
    sprintf(device_cmd, "%s\r\n\0", "E013");
    HAL_UART_Transmit_DMA(&huart1, device_cmd, strlen(device_cmd));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set technological signal state
 * @param line  - signal line number
 * @param state - signal state
 */
void M06 (char** args) {
    uint16_t mask  = 0x0001;
    uint16_t state = atoi(args[1]);
    uint16_t line  = atoi(args[0]);

    if (state) ts_state |= (mask << line);
    else ts_state &= !(mask << line);

    HAL_SPI_Transmit(tsgl_port, (uint8_t*)&ts_state, sizeof(uint16_t), 200);
}

/**
 * @brief Send command via rs-485 interface to external device (reserved for future use)
 * @param addr    - device address
 * @param command - command to send
 */
void C00 (char** args) {}

/**
 * @brief Receive answer via rs-485 interface (reserved for future use)
 */
void C01 (char** args) {}

/**
 * @brief Start programming sequence
 * @param start_block    - start block number to place programm
 * @param start_sector   - start sector number 
 * @param start_page     - start page number
 */
void W00 (char** args) {
    controller_state      = PROGRAMMING;
    uint16_t start_block  = atoi(args[0]);
    uint16_t start_sector = atoi(args[1]);
    uint16_t start_page   = atoi(args[2]);

    PROGRAMM_BLOCK   = start_block;
    PROGRAMM_SECTOR  = start_sector;
    PROGRAMM_PAGE    = start_page;
}

/**
 * @brief Stop programming sequence
 */
void W01 (char** args) {
    controller_state = INIT;
}

/**
 * @brief End of programm marker
 */
void M30 (char** args) {
    controller_state = INIT;
}

/**
 * @brief Start programm from specified address for specified cycles (interpreter entry point)
 * @param start_addr   - starting address
 * @param num_cycles   - desired cycles of programm execution
 */
void TrinityRun (char** args) {
    uint32_t start_addr = atoi(args[0]);
    uint32_t PROGRAM_CYCLES = atoi(args[1]);
    
    // Get block, page and sector where program is located
    PROGRAMM_BLOCK  = start_addr / 10000;
    PROGRAMM_SECTOR = (start_addr / 100) % 100;
    PROGRAMM_PAGE   = start_addr % 100;
    
    int page_iterator = 0;    
    //-------------------------------------------------------------------------
    // PROGRAMM EXECUTION CYCLE
    //-------------------------------------------------------------------------
    for (int i = 0; i < PROGRAM_CYCLES; ++i) {
        controller_state = RUN;
        while (controller_state != INIT) {
            // 1. Load page
            memset(page_buffer, 0x00, PAGE_SIZE);
            TrinityReadPage(PROGRAMM_BLOCK, PROGRAMM_SECTOR, (PROGRAMM_PAGE + page_iterator), (uint8_t*)&page_buffer, PAGE_SIZE);
            // 2. Command execution cycle
            itp->peek(itp);
            while (itp->exec(itp)) itp->peek(itp);
            page_iterator++;
        }
        page_iterator = 0;
    }
}

/**
 * @brief Bind X-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
void W02 (char** args) {
    TrinityLoadSettings();
    uint16_t selector  = atoi(args[0]);
    uint16_t sens_line = atoi(args[1]);
    
    if (!selector) controller_settings.X_MIN_POS_SENSOR_MASK = sens_line;
    else controller_settings.X_MAX_POS_SENSOR_MASK = sens_line;

    TrinitySaveSettings();
}

/**
 * @brief Bind Y-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
void W03 (char** args) {
    TrinityLoadSettings();
    uint16_t selector  = atoi(args[0]);
    uint16_t sens_line = atoi(args[1]);

    if (!selector) controller_settings.Y_MIN_POS_SENSOR_MASK = sens_line;
    else controller_settings.Y_MAX_POS_SENSOR_MASK = sens_line;

    TrinitySaveSettings();
}

/**
 * @brief Bind Z-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
void W04 (char** args) {
    TrinityLoadSettings();
    uint16_t selector  = atoi(args[0]);
    uint16_t sens_line = atoi(args[1]);

    if (!selector) controller_settings.Z_MIN_POS_SENSOR_MASK = sens_line;
    else controller_settings.Z_MAX_POS_SENSOR_MASK = sens_line;

    TrinitySaveSettings();
}

/**
 * @brief Bind A-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
void W05 (char** args) {
    TrinityLoadSettings();
    uint16_t selector  = atoi(args[0]);
    uint16_t sens_line = atoi(args[1]);

    if (!selector) controller_settings.A_MIN_POS_SENSOR_MASK = sens_line;
    else controller_settings.A_MAX_POS_SENSOR_MASK = sens_line;

    TrinitySaveSettings();
}

/**
 * @brief Set axises activities
 * @param x_active - X activity to be set
 * @param y_active - y activity to be set
 * @param z_active - z activity to be set
 * @param a_active - a activity to be set
 */
void W06 (char** args) {
    HAL_StatusTypeDef res = HAL_OK;

    control_struct.X_AXIS_SON = atoi(args[0]);
    control_struct.Y_AXIS_SON = atoi(args[1]);
    control_struct.Z_AXIS_SON = atoi(args[2]);
    control_struct.A_AXIS_SON = atoi(args[3]);

    res = HAL_SPI_Transmit(srvo_port, (uint8_t*)&control_struct, sizeof(SERVO_CONTROL_STRUCTURE), 200);

    return (uint32_t)res;
}

/**
 * @brief Set axises emergency conditions
 * @param x_active - X emergemcy condition
 * @param y_active - y emergency condition
 * @param z_active - z emergency condition
 * @param a_active - a emergency condition
 */
void W07 (char** args) {
    HAL_StatusTypeDef res = HAL_OK;

    control_struct.X_AXIS_EMGS = atoi(args[0]);
    control_struct.Y_AXIS_EMGS = atoi(args[1]);
    control_struct.Z_AXIS_EMGS = atoi(args[2]);
    control_struct.A_AXIS_EMGS = atoi(args[3]);

    res = HAL_SPI_Transmit(srvo_port, (uint8_t*)&control_struct, sizeof(SERVO_CONTROL_STRUCTURE), 200);

    return (uint32_t)res;
}

/**
 * @brief Wait for specified sensor line become active
 * @param sens_line - sensor line number to be check
 */
void W08 (char** args) {
    uint16_t sens_line = atoi(args[0]);
    while (!TrinityGetSensorState(sens_line)) HAL_Delay(1);
}

/**
 * @brief Wait for specified sensor line become passive
 * @param sens_line - sensor line number to be checked
 */
void W09 (char** args) {
    uint16_t sens_line = atoi(args[0]);
    while (TrinityGetSensorState(sens_line)) HAL_Delay(1);
}

//-----------------------------------------------------------------------------
// INTERRUPT SERVICE CALLBACKS
//-----------------------------------------------------------------------------
void onEmapCommandReceived(void) {
    HAL_UART_AbortReceive_IT(&huart1);
    EMAP_ACK_RECEIVED = true;
}

 void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
     if (GPIO_Pin == SERVO_X_SRDY_INT_Pin) {
         servo_x_ready = true;
     } else if (GPIO_Pin == SERVO_Y_SRDY_INT_Pin) {
         servo_y_ready = true;
     } else if (GPIO_Pin == SERVO_Z_SRDY_INT_Pin) {
         servo_z_ready = true;
     } else if (GPIO_Pin == SERVO_A_SRDY_INT_Pin) {
         servo_a_ready = true;
     } else if (GPIO_Pin == SENS_INTERRUPT_Pin) {
         if (TrinityGetSensorState(controller_settings.X_MAX_POS_SENSOR_MASK) || TrinityGetSensorState(controller_settings.X_MIN_POS_SENSOR_MASK)) {
             HAL_TIM_Base_Stop_IT(&htim8);
             HAL_TIM_Base_Stop_IT(&htim5);
             op_complete_X = true;
         } else if (TrinityGetSensorState(controller_settings.Y_MAX_POS_SENSOR_MASK) || TrinityGetSensorState(controller_settings.Y_MIN_POS_SENSOR_MASK)) {
             HAL_TIM_Base_Stop_IT(&htim1);
             HAL_TIM_Base_Stop_IT(&htim4);
             op_complete_Y = true;   
         } else if (TrinityGetSensorState(controller_settings.Z_MAX_POS_SENSOR_MASK) || TrinityGetSensorState(controller_settings.Z_MIN_POS_SENSOR_MASK)) {
             HAL_TIM_Base_Stop_IT(&htim2);
             HAL_TIM_Base_Stop_IT(&htim3);
             op_complete_Z = true;
         } else if (TrinityGetSensorState(controller_settings.A_MAX_POS_SENSOR_MASK) || TrinityGetSensorState(controller_settings.A_MIN_POS_SENSOR_MASK)) {
             HAL_TIM_Base_Stop_IT(&htim9);
             HAL_TIM_Base_Stop_IT(&htim10);
             op_complete_A = true;
         }
     }
 }
//=============================================================================
