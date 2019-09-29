#include "command.h"

UART_HandleTypeDef* emap_port = &huart1;
UART_HandleTypeDef* extm_port = &huart2;
SPI_HandleTypeDef*  eprm_port = &hspi1;
SPI_HandleTypeDef*  srvo_port = &hspi2;
SPI_HandleTypeDef*  tsgl_port = &hspi3;

//=============================================================================
// TRINITY SERVO CONTROLLER GLOBAL VATIABLES
//=============================================================================
static double  distance_x, distance_y, distance_z, distance_a;
static uint8_t dir_x, dir_y, dir_z, dir_a;

static bool    op_complete_X = false;
static bool    op_complete_Y = false;
static bool    op_complete_Z = false;
static bool    op_complete_A = false;

static uint16_t ts_state     = 0x0000;
static uint16_t sensor_state = 0x0000;

static uint16_t device_addr;
static uint8_t* device_command;
static uint32_t program_start_addr;
static uint8_t* program_command;

uint8_t (*cmd[47])(void) = {G0000, G0001, G0002, G0003, G0004, G0005, G0006, G0007, G0008, G0009, 
                            G0010, G0011, G0012, G0013, G0014, G0015, G0016, G0017, G0018, G0019, 
                            G0020, G0021, G0022, G0023, G0024, G0025, G0026, G0027, G0028, G0029, 
                            G0030, G0031, G0032, G0033, G0034, G0035, G0036, G0037, G0038, G0039, 
                            G0040, G0041, G0042, G0043, G0044, G0045, G0046};

// SERVO SUBSYSTEM STATE. !!!IMPIORTANT!!! MUST be transmitted to subsystem in MSB order
static struct {
    uint32_t A_AXIS_EMGS: 1;
    uint32_t A_AXIS_CCLR: 1;
    uint32_t A_AXIS_SPD0: 1;
    uint32_t A_AXIS_DIR : 1;
    uint32_t A_AXIS_SON : 1;
    uint32_t A_AXIS_S_P : 1;
    uint32_t A_AXIS_SPD1: 1;
    uint32_t A_AXIS_IND : 1;

    uint32_t Z_AXIS_EMGS: 1;
    uint32_t Z_AXIS_CCLR: 1;
    uint32_t Z_AXIS_SPD0: 1;
    uint32_t Z_AXIS_DIR : 1;
    uint32_t Z_AXIS_SON : 1;
    uint32_t Z_AXIS_S_P : 1;
    uint32_t Z_AXIS_SPD1: 1;
    uint32_t Z_AXIS_IND : 1;

    uint32_t Y_AXIS_EMGS: 1;
    uint32_t Y_AXIS_CCLR: 1;
    uint32_t Y_AXIS_SPD0: 1;
    uint32_t Y_AXIS_DIR : 1;
    uint32_t Y_AXIS_SON : 1;
    uint32_t Y_AXIS_S_P : 1;
    uint32_t Y_AXIS_SPD1: 1;
    uint32_t Y_AXIS_IND : 1;

    uint32_t X_AXIS_EMGS: 1;
    uint32_t X_AXIS_CCLR: 1;
    uint32_t X_AXIS_SPD0: 1;
    uint32_t X_AXIS_DIR : 1;
    uint32_t X_AXIS_SON : 1;
    uint32_t X_AXIS_S_P : 1;
    uint32_t X_AXIS_SPD1: 1;
    uint32_t X_AXIS_IND : 1;
} SERVO_STATE;    

void TrinityCPInit(void) {
    device_command  = (uint8_t*)calloc(PGM_BUFFER_SIZE, sizeof(uint8_t));
    program_command = (uint8_t*)calloc(PGM_BUFFER_SIZE, sizeof(uint8_t));
}

void TrinityCommandExec(char* command) {
    
}

void TrinityGetCommandCode(char* command) {
    
}

void TrinityGetCommandArgs(char* command) {
    
}

uint8_t G0000(void) {
    return 0;
}

uint8_t G0001(void) {
    return 0;    
}

uint8_t G0002(void) {
    return 0;    
}

uint8_t G0003(void) {
    return 0;    
}

uint8_t G0004(void) {
    return 0;     
}

uint8_t G0005(void) {
    return 0;     
}

uint8_t G0006(void) {
    return 0;     
}

uint8_t G0007(void) {
    return 0;     
}

uint8_t G0008(void) {
    return 0;     
}

uint8_t G0009(void) {
    return 0;     
}

uint8_t G0010(void) {
    return 0;     
}

uint8_t G0011(void) {
    return 0;     
}

uint8_t G0012(void) {
    return 0;     
}

uint8_t G0013(void) {
    return 0;     
}

uint8_t G0014(void) {
    return 0;     
}

uint8_t G0015(void) {
    return 0;     
}

uint8_t G0016(void) {
    return 0;     
}

uint8_t G0017(void) {
    return 0;     
}

uint8_t G0018(void) {
    return 0;     
}

uint8_t G0019(void) {
    return 0;     
}

uint8_t G0020(void) {
    return 0; 
}

uint8_t G0021(void) {
    return 0;     
}

uint8_t G0022(void) {
    return 0;     
}

uint8_t G0023(void) {
    return 0;
}

uint8_t G0024(void) {
    return 0;
}

uint8_t G0025(void) {
    return 0;    
}

uint8_t G0026(void) {
    return 0;    
}

uint8_t G0027(void) {
    return 0;    
}

uint8_t G0028(void) {
    return 0;    
}

uint8_t G0029(void) {
    return 0;    
}

uint8_t G0030(void) {
    return 0;    
}

uint8_t G0031(void) {
    return 0;    
}

uint8_t G0032(void) {
    return 0;    
}

uint8_t G0033(void) {
    return 0;    
}

uint8_t G0034(void) {
    return 0;    
}

uint8_t G0035(void) {
    return 0;    
}

uint8_t G0036(void) {
    return 0;    
}

uint8_t G0037(void) {
    return 0;    
}

uint8_t G0038(void) {
    return 0;    
}

uint8_t G0039(void) {
    return 0;    
}

uint8_t G0040(void) {
    return 0;    
}

uint8_t G0041(void) {
    return 0;    
}

uint8_t G0042(void) {
    return 0;    
}

uint8_t G0043(void) {
    return 0;    
}

uint8_t G0044(void) {
    return 0;    
}

uint8_t G0045(void) {
    return 0;    
}

uint8_t G0046(void) {
    return 0;    
}


//=============================================================================
// SERVO PULSE TIMERS interrupt serving callback
//=============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    __HAL_TIM_DISABLE(htim);
    
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



