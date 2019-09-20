#include "process.h"

static UART_HandleTypeDef* emap_port = &huart1;
static void (*process[31])(void) = {E000, E001, E002, E003, E004, E005, E006, E007, E008, E009, E010, E011, E012, E013};

//=============================================================================
// MACHINE STATE INTEGRAL INDICATOR
//=============================================================================
static MACHINE_STATE emap_state;
static uint8_t machine_state_string[64];

//=============================================================================
// Common use data
//=============================================================================
static uint8_t emap_buffer[BUFFER_SIZE];
static char cmd_ack_string[10];
static uint8_t emap_cmd_code;
static bool emap_command_received = false;

//=============================================================================
// EMAP auxiliary routines
//=============================================================================
void EmapCommandRead(void) {
    emap_command_received = false;
    memset(emap_buffer, '\0', BUFFER_SIZE);
    HAL_UART_Receive_DMA(emap_port, emap_buffer, BUFFER_SIZE);
}

void onEmapCommandReceived(void) {
    HAL_UART_DMAStop(emap_port);
    emap_command_received = true; 
}

bool EmapIsCommandReceived(void) {
    return emap_command_received;
}

static void EmapCommandParse(void) {
    char code[3] = {'\0', '\0', '\0'};
    memcpy(code, strchr((char*)emap_buffer, 'E') + 1, 3);
    emap_cmd_code = atoi(code);        
}

static uint8_t EmapGetCommandCode(void) {
    return emap_cmd_code;
}

static void EmapCommandAck(bool cmd_result) {
    memset(cmd_ack_string, 0x00, 10);
    if (cmd_result) sprintf(cmd_ack_string, "OK\r\n");
    else            sprintf(cmd_ack_string, "ERROR\r\n");
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)cmd_ack_string, strlen(cmd_ack_string));    
}

static void EmapStateCheck(void) {
    GPIO_PinState line_state;
    
    // MACHINE_EMGS state check
    line_state = HAL_GPIO_ReadPin(MACHINE_EMGS_GPIO_Port, MACHINE_EMGS_Pin);
    if (line_state) emap_state.MACHINE_EMGS = true;
    else            emap_state.MACHINE_EMGS = false;
    
    // MOULD_OPEN state check
    line_state = HAL_GPIO_ReadPin(MOULD_OPEN_POS_GPIO_Port, MOULD_OPEN_POS_Pin);
    if (line_state) emap_state.MOULD_OPEN = false;
    else            emap_state.MOULD_OPEN = true;

    // MACHINE_SAFETY state check
    line_state = HAL_GPIO_ReadPin(MACHINE_SAFETY_GPIO_Port, MACHINE_SAFETY_Pin);
    if (line_state) emap_state.MACHINE_SAFETY = false;
    else            emap_state.MACHINE_SAFETY = true;

    // REJECT state check
    line_state = HAL_GPIO_ReadPin(REJECT_GPIO_Port, REJECT_Pin);
    if (line_state) emap_state.REJECT = false;
    else            emap_state.REJECT = true;

    // DEVICE_OP_ENA state check
    line_state = HAL_GPIO_ReadPin(DEVICE_OP_ENA_GPIO_Port, DEVICE_OP_ENA_Pin);
    if (line_state) emap_state.DEVICE_OP_ENA = false;
    else            emap_state.DEVICE_OP_ENA = true;
    
    // MOULD_CLOSED state check
    line_state = HAL_GPIO_ReadPin(MOULD_CLOSED_GPIO_Port, MOULD_CLOSED_Pin);
    if (line_state) emap_state.MOULD_CLOSED = false;
    else            emap_state.MOULD_CLOSED = true;
    
    // INTER_MOULD_OPEN state check
    line_state = HAL_GPIO_ReadPin(INTER_OPEN_POS_GPIO_Port, INTER_OPEN_POS_Pin);
    if (line_state) emap_state.INTER_MOULD_OPEN = false;
    else            emap_state.INTER_MOULD_OPEN = true;
    
    // EJECTOR_IN_BCK_POS state check
    line_state = HAL_GPIO_ReadPin(EJECT_IN_BACK_POS_GPIO_Port, EJECT_IN_BACK_POS_Pin);
    if (line_state) emap_state.EJECTOR_IN_BCK_POS = false;
    else            emap_state.EJECTOR_IN_BCK_POS = true;
    
    // EJECTOR_IN_FWD_POS state check
    line_state = HAL_GPIO_ReadPin(EJECT_IN_FWD_POS_GPIO_Port, EJECT_IN_FWD_POS_Pin);
    if (line_state) emap_state.EJECTOR_IN_FWD_POS = false;
    else            emap_state.EJECTOR_IN_FWD_POS = true;
    
    // CORE_PULLERS_IN_POS1 state check
    line_state = HAL_GPIO_ReadPin(COREPULLER_POS1_GPIO_Port, COREPULLER_POS1_Pin);
    if (line_state) emap_state.CORE_PULLERS_IN_POS1 = false;
    else            emap_state.CORE_PULLERS_IN_POS1 = true;
    
    // CORE_PULLERS_IN_POS2 state check
    line_state = HAL_GPIO_ReadPin(COREPULLER_POS2_GPIO_Port, COREPULLER_POS2_Pin);
    if (line_state) emap_state.CORE_PULLERS_IN_POS2 = false;
    else            emap_state.CORE_PULLERS_IN_POS2 = true;
}

static void EmapSendState(void) {
    memset(machine_state_string, 0x00, 64);
    sprintf((char*)machine_state_string, "%u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u\r\n",
            emap_state.MACHINE_EMGS, emap_state.MOULD_OPEN, emap_state.MACHINE_SAFETY, emap_state.REJECT, emap_state.DEVICE_OP_ENA,
            emap_state.MOULD_CLOSED, emap_state.INTER_MOULD_OPEN, emap_state.EJECTOR_IN_BCK_POS, emap_state.EJECTOR_IN_FWD_POS,
            emap_state.CORE_PULLERS_IN_POS1, emap_state.CORE_PULLERS_IN_POS2);   
    HAL_UART_Transmit_DMA(&huart1, machine_state_string, strlen((char*)machine_state_string));    
}
static void EmapInit(void) {
    // Enabling handling device operation, setting mould area free and dropping emergency conditions
    E000();
    E005();
    E012();
    // Checking initial moulding machine state
    EmapStateCheck();
    // Moulding machine state check timer start
    HAL_TIM_Base_Start_IT(&htim6);    
}

//=============================================================================
// EMAP COMMAND RECEIVER INTERRUPT CALLBACK
//=============================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) EmapStateCheck();
}

//=============================================================================
// EMAP COMMAND PROCESSING ROUTINES
//=============================================================================
void EmapCommandProcessing(void) {
    EmapCommandParse();
    process[EmapGetCommandCode()]();
}

static void E000(void) {
    HAL_GPIO_WritePin(DEVICE_OP_MODE_GPIO_Port, DEVICE_OP_MODE_Pin, GPIO_PIN_SET);
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);
}

static void E001(void) {
    HAL_GPIO_WritePin(DEVICE_OP_MODE_GPIO_Port, DEVICE_OP_MODE_Pin, GPIO_PIN_RESET);
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);
}

static void E002(void) {
    // Wait until mould is open
    while (!emap_state.MOULD_OPEN);
    // Disabling mould closure and setting mould area occupied condition
    E011();
    E013();
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);
}

static void E003(void) {
    E012();
    E010();
}

static void E004(void) {
    HAL_GPIO_WritePin(DEVICE_EMGS_GPIO_Port, DEVICE_EMGS_Pin, GPIO_PIN_SET);  
    HAL_Delay(COMMAND_TIMEOUT);    
    EmapCommandAck(true);
}

static void E005(void) {
    HAL_GPIO_WritePin(DEVICE_EMGS_GPIO_Port, DEVICE_EMGS_Pin, GPIO_PIN_SET);   
    HAL_Delay(COMMAND_TIMEOUT);    
    EmapCommandAck(true);
}

static void E006(void) {
    
}

static void E007(void) {
    
}

static void E008(void) {
    
}

static void E009(void) {
    
}

static void E010(void) {
    // Enabling mould closure
    HAL_GPIO_WritePin(MOLD_CLOSE_ENA_GPIO_Port, MOLD_CLOSE_ENA_Pin, GPIO_PIN_RESET);    
    while (!emap_state.MOULD_CLOSED);
    HAL_Delay(COMMAND_TIMEOUT);
    HAL_GPIO_WritePin(MOLD_CLOSE_ENA_GPIO_Port, MOLD_CLOSE_ENA_Pin, GPIO_PIN_SET);       
    EmapCommandAck(true);    
}

static void E011(void) {
    // Disabling mould closure
    HAL_GPIO_WritePin(MOLD_CLOSE_ENA_GPIO_Port, MOLD_CLOSE_ENA_Pin, GPIO_PIN_SET);     
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);    
}

static void E012(void) {
    // Mould area IS FREE!
    HAL_GPIO_WritePin(MOLD_AREA_FREE_GPIO_Port, MOLD_AREA_FREE_Pin, GPIO_PIN_RESET);      
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);    
}

static void E013(void) {
    // Mould area IS OCCUPIED BY HANDLING DEVICE!
    HAL_GPIO_WritePin(MOLD_AREA_FREE_GPIO_Port, MOLD_AREA_FREE_Pin, GPIO_PIN_SET);   
    HAL_Delay(COMMAND_TIMEOUT);
    EmapCommandAck(true);    
}



