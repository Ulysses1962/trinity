#include "control.h"
#include "can.h"

// Количество мм на один импульс команды
#define RESOLUTION 0.01      
#define MACHINE_ZERO_TRESHOLD 50.00
#define MACHINE_ZERO_DISTANCE 20000.00

// Operation flags and variables
static volatile bool MAX_POS_REACHED    = false;
static volatile bool MIN_POS_REACHED    = false;
static volatile bool FIRST_START        = true;


// --------------------------------------------------------------------
// CAN messages
//---------------------------------------------------------------------
const uint32_t SERVO_ALARM          = 0x00E0; // - ALARM message
const uint32_t SERVO_FORWARD        = 0x00E1; // - MOVE FORWARD message
const uint32_t SERVO_BACKWARD       = 0x00E2; // - MOVE BACKWARDS message
const uint32_t SERVO_HOME           = 0x00E3; // - MOVE HOME message
const uint32_t SERVO_INIT           = 0x00E4; // - INIT SERVO CONTROLLER message
const uint32_t SERVO_SRDY           = 0x00E5; // - SRDY message
const uint32_t SERVO_NSRDY          = 0x00E6; // - DRIVE NOT READY message
const uint32_t SERVO_TPOS           = 0x00E7; // - TPOS message
const uint32_t SERVO_HOME_READY     = 0x00E8; // - HOME_READY messageextern const uint32_t SERVO_ALARM;              // - ALARM message

static CanTxMsgTypeDef msg;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    TIM2->CR1 &= ~ TIM_CR1_CEN;
    if (drive_state == DRIVE_ON_DUTY) on_servo_command_complete();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (drive_state == DRIVE_ZERO_MODE) {
        if (GPIO_Pin == MAX_POS_Pin) {
            TIM2->EGR |= TIM_EGR_UG;
            MAX_POS_REACHED = true;
        } else if (GPIO_Pin == MIN_POS_Pin) {
            TIM2->EGR |= TIM_EGR_UG;
            MIN_POS_REACHED = true;
        }
    } else if (drive_state == DRIVE_ON_DUTY) {
        if (GPIO_Pin == ALARM_Pin || GPIO_Pin == MAX_POS_Pin || GPIO_Pin == MIN_POS_Pin) servo_emgs(true);
    }
}

void process_console(char* message) {
    if (*message != ':') {
        servo_cmd_error = true;
        Error_Handler();
        return;
    }

    double distance;
    int direction;
    
    switch(*(message + 1)) {
        case '&':
            if (drive_state == DRIVE_ACTIVE) servo_machine_zero(); 
        break;
        case 'i': 
            if (drive_state == DRIVE_INACTIVE) servo_init(true); 
        break;
        case '+': 
            if (drive_state == DRIVE_ON_DUTY) {
                direction = 1;
                distance  = atof(message + 2);
                servo_move(distance, direction);
            }
        break;
        case '-':
            if (drive_state == DRIVE_ON_DUTY) {
                direction = 0;
                distance  = atof(message + 2);
                servo_move(distance, direction);
            }
        break;
        case 'r':
            if (drive_state == DRIVE_ACTIVE || drive_state == DRIVE_ON_DUTY) servo_init(false); 
        break;  
        default:
            servo_cmd_error = true;
            Error_Handler();
            break;
    }
}

void process_can(CanRxMsgTypeDef* message) {
    switch (message->StdId) {
        case SERVO_BACKWARD:
            console_send("SERVO-BACKWARD message received!\r\n");
            if (drive_state == DRIVE_ON_DUTY) servo_move(atof((char *)message->Data), 0);
        break;
        case SERVO_FORWARD:
            console_send("SERVO-FORWARD message received!\r\n");
            if (drive_state == DRIVE_ON_DUTY) servo_move(atof((char *)message->Data), 1);
        break;
        case SERVO_INIT:
            console_send("SERVO-INIT message received!\r\n");
            if (message->Data[0] == 1 && drive_state == DRIVE_INACTIVE) servo_init(true);
            if (message->Data[0] == 0 && (drive_state == DRIVE_ACTIVE || drive_state == DRIVE_ON_DUTY)) servo_init(false);
        break;
        case SERVO_HOME:
            console_send("SERVO-HOME message received!\r\n");
            if (drive_state == DRIVE_ACTIVE || drive_state == DRIVE_ON_DUTY) servo_machine_zero();
        break;
    }
}


static void servo_move(double dist, int dir) {
    if (dist < RESOLUTION) {
        servo_dst_error = true;
        Error_Handler();
        return;
    }
    
    HAL_GPIO_WritePin(HOME_IND_GPIO_Port, HOME_IND_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TPOS_IND_GPIO_Port, TPOS_IND_Pin, GPIO_PIN_RESET);

    // Вычисляем количество импульсов
    double n_pulse;
    modf(dist / RESOLUTION, &n_pulse);   
    // Устанавливаем флаг направления
    if (!dir) {
        HAL_GPIO_WritePin(FORWARD_GPIO_Port, FORWARD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BACKWARD_GPIO_Port, BACKWARD_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BACKWARD_GPIO_Port, BACKWARD_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FORWARD_GPIO_Port, FORWARD_Pin, GPIO_PIN_SET);
    }
    
    // Запускаем генератор команды
    if (FIRST_START) {
        htim2.Init.Period = (uint32_t)n_pulse;
        if (HAL_TIM_Base_Init(&htim2) != HAL_OK) _Error_Handler(__FILE__, __LINE__);
        FIRST_START = false;
    } else {
        TIM2->ARR = (uint32_t)n_pulse; 
    }
    //servo_cclr();
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void servo_init(bool active) {   
    msg.IDE   = CAN_ID_STD;
    msg.RTR   = CAN_RTR_DATA;
    msg.DLC   = 0;
    
    if (!active) {
        HAL_GPIO_WritePin(SON_GPIO_Port, SON_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SRDY_IND_GPIO_Port, SRDY_IND_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(TPOS_IND_GPIO_Port, TPOS_IND_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(HOME_IND_GPIO_Port, HOME_IND_Pin, GPIO_PIN_RESET);
        // Setting inactive state
        drive_state = DRIVE_INACTIVE;
        msg.StdId = SERVO_NSRDY;
    } else if (HAL_GPIO_ReadPin(GPIOB, SRDY_Pin) == GPIO_PIN_SET) {
        // Activating SON line
        HAL_GPIO_WritePin(SON_GPIO_Port, SON_Pin, GPIO_PIN_SET);
        // Set SRDY indicator
        HAL_GPIO_WritePin(SRDY_IND_GPIO_Port, SRDY_IND_Pin, GPIO_PIN_SET);
        // Setting drive state
        drive_state = DRIVE_ACTIVE;
        // Sending active state acknoledge
        msg.StdId = SERVO_SRDY;
    }
    can_send_message(&msg);    
}

void servo_emgs(bool active) {
    if (active) {
        HAL_GPIO_WritePin(EMGS_GPIO_Port, EMGS_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ALARM_IND_GPIO_Port, ALARM_IND_Pin, GPIO_PIN_SET);
        drive_state = DRIVE_INACTIVE;
        // Send ALARM to CPU.
        msg.IDE   = CAN_ID_STD;
        msg.StdId = SERVO_ALARM;
        msg.RTR   = CAN_RTR_DATA;
        msg.DLC   = 0;
        can_send_message(&msg);
        
    } else {
        HAL_GPIO_WritePin(EMGS_GPIO_Port, EMGS_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(ALARM_IND_GPIO_Port, ALARM_IND_Pin, GPIO_PIN_RESET);
    }
}

static void servo_machine_zero() {
    drive_state = DRIVE_ZERO_MODE;
    MAX_POS_REACHED   = false;
    MIN_POS_REACHED   = false;
    
    // Setting machine zero search frequency
    htim1.Init.Period = pwmz_period;
    HAL_TIM_Base_Init(&htim1);
    htim2.Init.Prescaler = trgm_prescaler; 
    HAL_TIM_Base_Init(&htim2);
    
    
    servo_move(MACHINE_ZERO_DISTANCE, 1);
    while (!MAX_POS_REACHED);
    servo_move(MACHINE_ZERO_DISTANCE, 0);
    while (!MIN_POS_REACHED);

    // Restoring frequency settings    
    htim1.Init.Period = pwm_period;
    HAL_TIM_Base_Init(&htim1);
    htim2.Init.Prescaler = trg_prescaler; 
    HAL_TIM_Base_Init(&htim2);

    servo_move(MACHINE_ZERO_TRESHOLD, 1);
    HAL_GPIO_WritePin(HOME_IND_GPIO_Port, HOME_IND_Pin, GPIO_PIN_SET);
    drive_state = DRIVE_ON_DUTY;
    MAX_POS_REACHED   = false;
    MIN_POS_REACHED   = false;
       
    // Sending HOME message to CPU.   
    msg.IDE   = CAN_ID_STD;
    msg.StdId = SERVO_HOME_READY;
    msg.RTR   = CAN_RTR_DATA;
    msg.DLC   = 0;
    can_send_message(&msg);    
}

//--------------------------------------------------------------------------
// Servo interrupt service routines
//--------------------------------------------------------------------------
void on_servo_command_complete(void) {
    HAL_GPIO_WritePin(TPOS_IND_GPIO_Port, TPOS_IND_Pin, GPIO_PIN_SET);
    msg.IDE   = CAN_ID_STD;
    msg.StdId = SERVO_TPOS;
    msg.RTR   = CAN_RTR_DATA;
    msg.DLC   = 0;
    can_send_message(&msg);
}

