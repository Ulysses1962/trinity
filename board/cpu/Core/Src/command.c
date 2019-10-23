#include "command.h"
#include "memory.h"

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

static const uint8_t PROGRAMM_BLOCK         = 1;
static const uint8_t PROGRAMM_SECTOR        = 0;
static const uint8_t PROGRAMM_PAGE          = 0;                             

//-----------------------------------------------------------------------------
// TRINITY common interface routines
//-----------------------------------------------------------------------------
void TrinityCPInit(void) {
    device_command  = (char*)calloc(PGM_BUFFER_SIZE, sizeof(uint8_t));
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

void TrinityCommandParser(char* command) {
    
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

//=============================================================================
// Utility routines
//=============================================================================
uint32_t TrinitySetAxisDirection(TRINITY_AXIS axis, TRINITY_SERVO_DIRECTION dir) {
    HAL_StatusTypeDef res = HAL_OK;

    switch (axis) {
        case CH_A: control_struct.A_AXIS_DIR = dir; break;
        case CH_Z: control_struct.Z_AXIS_DIR = dir; break;
        case CH_Y: control_struct.Y_AXIS_DIR = dir; break;
        case CH_X: control_struct.X_AXIS_DIR = dir; break;
    }
    res = HAL_SPI_Transmit(tsgl_port, (uint8_t*)&control_struct, sizeof(SERVO_CONTROL_STRUCTURE), 200);

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
static void G00 (double x, double y, double z, double a) {
    if (!DEVICE_ALARM) return;

    G00_X(x, controller_settings.X_MAX_FEED);
    G00_Y(y, controller_settings.Y_MAX_FEED);
    G00_Z(z, controller_settings.Z_MAX_FEED);
    G00_A(a, controller_settings.A_MAX_FEED);
}

/** 
 * @brief Moves along X-axis to specified position with specified feed in mm/min
 * @param x    - X-coord of specified point
 * @param feed - movement feed in mm/min
 */
static void G00_X (double x, double feed) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dx;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_X, feed);

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
static void G00_Y (double y, double feed) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dy;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_Y, feed);
    
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
static void G00_Z (double z, double feed) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double dz;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_Z, feed);

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
static void G00_A (double a, double feed) {
    if (!DEVICE_ALARM) return;

    uint16_t n_pulse;
    double da;
    TRINITY_SERVO_DIRECTION dir;

    TrinitySetAxisSpeed(CH_A, feed);

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
static void G01 (double x, double y, double z, double a, double feed) {
    if (!DEVICE_ALARM) return;

    G00_X(x, feed);
    G00_Y(y, feed);
    G00_Z(z, feed);
    G00_A(a, feed);    
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
static void G02 (double x, double y, double z, double r, double feed) {}

/**
 *  @brief  Circular CCW interpolation to specified point using specified radius and speed
 *  @param x    - x coordinate of destination point
 *  @param y    - y coordinate of destination point
 *  @param z    - z coordinate of destination point
 *  @param r    - interpolation radius 
 *  @param feed - feed speed mm/min
 *  Reserved for future use
 */
static void G03 (double x, double y, double z, double r, double feed) {}

/**
 *  @brief  Programm delay for specified duration
 *  @param m_delay - delay in milliseconds
 */
static void G04 (uint32_t m_delay) {
    while (!(op_complete_A && op_complete_X && op_complete_Y && op_complete_Z));
    HAL_Delay(m_delay);
}

/**
 *  @brief  Set reference point
 *  @param x    - x-coordinate of reference point
 *  @param y    - y-coordinate of reference point
 *  @param z    - z-coordinate of reference point
 *  @param a    - a-coordinate of reference point
 */
static void G20 (double x, double y, double z, double a) {
    controller_settings.X_REF_COORD = x;
    controller_settings.Y_REF_COORD = y;
    controller_settings.Z_REF_COORD = z;
    controller_settings.A_REF_COORD = a;

    TrinitySaveSettings();
}

/**
 *  @brief  Set axis resolutions
 *  @param res_x    - x axis resolution
 *  @param res_y    - y axis resolution
 *  @param res_z    - z axis resolution
 *  @param res_a    - a axis resolution
 */
static void G21 (double res_x, double res_y, double res_z, double res_a) {
    controller_settings.X_AXIS_RESOLUTION = res_x;
    controller_settings.Y_AXIS_RESOLUTION = res_y;
    controller_settings.Z_AXIS_RESOLUTION = res_z;
    controller_settings.A_AXIS_RESOLUTION = res_a;

    TrinitySaveSettings(); 
}

/**
 *  @brief Return to reference point via the intermediate point
 *  @param x    - x axis intermediate coordinate
 *  @param y    - y axis intermediate coordinate
 *  @param z    - z axis intermediate coordinate
 *  @param a    - a axis intermediate coordinate
 */
static void G28 (double x, double y, double z, double a) {
    G00(x, y, z, a);
    G00(controller_settings.X_REF_COORD, controller_settings.Y_REF_COORD, controller_settings.Z_REF_COORD, controller_settings.A_REF_COORD);
}

/**
 * @brief Use absolute coordinates
 */
static void G90 (void) {
    coord_system = ABSOLUTE;
}

/**
 * @brief Use relative coordinates
 */
static void G91 (void) {
    coord_system = RELATIVE;
}

bool TrinityGetEMAPAck(void) {
   uint8_t resp[10];
   HAL_UART_Receive_IT(&huart1, resp, 10);

   while (!EMAP_ACK_RECEIVED);

   if (strstr(resp, "OK")) return true;
   else return false;
}

/**
 * @brief Device operation mode disable
 */
static void E01 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E001");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Start handling device operation cycle
 */
static void E02 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E002");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Finish handling device operation cycle
 */
static void E03 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E003");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set device emergency condition
 */
static void E04 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E004");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    TrinityDeviceAlarm(true);
}

/**
 * @brief Drop device emergency condition
 */
static void E05 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E005");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Move ejector to forward position
 */
static void E06 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E006");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Move ejector to back position
 */
static void E07 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E007");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set core pullers to position 2
 */
static void E08 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E008");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set core pullers to position 1
 */
static void E09 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E009");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Mould closure enable
 */
static void E10 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E010");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Mould closure disable
 */
static void E11 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E011");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set mould area free
 */
static void E12 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E012");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set mould area occupied
 */
static void E13 (void) {
    memset(device_command, 0x00, PGM_BUFFER_SIZE);
    sprintf(device_command, "%s\r\n", "E013");
    HAL_UART_Transmit_DMA(&huart1, device_command, strlen(device_command));

    bool emap_ack = TrinityGetEMAPAck();
    if (!emap_ack) TrinityDeviceAlarm(true);
}

/**
 * @brief Set technological signal state
 * @param line  - signal line number
 * @param state - signal state
 */
static void M06 (uint16_t line, uint8_t state) {

}

/**
 * @brief Send command via rs-485 interface to external device
 * @param addr    - device address
 * @param command - command to send
 */
static void C00 (uint16_t addr, char* command) {
    
}

/**
 * @brief Receive answer via rs-485 interface
 */
static void C01 (void) {

}

/**
 * @brief Start programming sequence
 * @param start_block    - start block number to place programm
 * @param start_sector   - start sector number 
 * @param start_page     - start page number
 */
static void W00 (uint16_t start_block, uint16_t start_sector, uint16_t start_page) {

}

/**
 * @brief Stop programming sequence
 */
static void W01 (void) {

}

/**
 * @brief Start programm from specified address for specified cycles
 * @param start_addr   - starting address
 * @param num_cycles   - desired cycles of programm execution
 */
static void M98 (uint32_t start_addr, uint32_t num_cycles) {

}

/**
 * @brief Bind X-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
static void W02 (uint8_t selector, uint16_t sens_line) {

}

/**
 * @brief Bind Y-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
static void W03 (uint8_t selector, uint16_t sens_line) {

}

/**
 * @brief Bind Z-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
static void W04 (uint8_t selector, uint16_t sens_line) {

}

/**
 * @brief Bind A-axis to sensor line
 * @param selector  - specifies what position is binded (0 - min position, 1 - max position)
 * @param sens_line - sensor line binded
 */
static void W05 (uint8_t selector, uint16_t sens_line) {

}

/**
 * @brief Set axises activities
 * @param x_active - X activity to be set
 * @param y_active - y activity to be set
 * @param z_active - z activity to be set
 * @param a_active - a activity to be set
 */
static void W06 (uint8_t x_active, uint8_t y_active, uint8_t z_active, uint8_t a_active) {

}

/**
 * @brief Set axises emergency conditions
 * @param x_active - X emergemcy condition
 * @param y_active - y emergency condition
 * @param z_active - z emergency condition
 * @param a_active - a emergency condition
 */
static void W07 (uint8_t x_emgs, uint8_t y_emgs, uint8_t z_emgs, uint8_t a_emgs) {

}

/**
 * @brief Wait for specified sensor line become active
 * @param sens_line - sensor line number to be checked
 * @return sensor state (true - sensor activated, false - sensor passive)
 */
static bool W08 (uint16_t sens_line) {

}

//-----------------------------------------------------------------------------
// INTERRUPT SERVICE CALLBACKS
//-----------------------------------------------------------------------------
void onEmapCommandReceived(void) {
    HAL_UART_AbortReceive_IT(&huart1);
    EMAP_ACK_RECEIVED = true;
}

