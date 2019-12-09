#include "wifi.h"
#include "console.h"

#define TCP_SEND_TRESHOLD 3

static UART_HandleTypeDef* wifi_port              = &huart3;
static volatile enum  receiver_state wifi_state   = START;
static volatile int   wifi_tcp_connect_id         = 100;
static volatile bool  wifi_cmd_result             = false;
static volatile char  wifi_response[WIFI_BUFFER_SIZE];
static volatile int   wifi_parser_position        = 0;

//-------------------------------------------------------------------------
// WIFI access point parameters
//-------------------------------------------------------------------------
static const char* ACCESS_POINT_NAME        = "ULYSSES ROBOTICS 00001N"; 
static const char* ACCESS_POINT_PASSWORD    = "123456789A";          
static const int TCP_SERVER_PORT            = 1212;                  
static const int TCP_SERVER_TIMEOUT         = 3600;    

//-------------------------------------------------------------------------
// WIFI initialization routines
//-------------------------------------------------------------------------
void wifi_init(void) {
    char* cmd = (char *)calloc(128, sizeof(char));
    for (int step = 1; step <= 8; ++step) {
        switch (step) {
            case 1: sprintf(cmd, "AT\r\n"); break;
            case 2: sprintf(cmd, "ATE0\r\n"); break;
            case 3: sprintf(cmd, "AT+CWMODE_DEF=%u\r\n", 3); break;
            case 4: sprintf(cmd, "AT+CWSAP_DEF=\"%s\",\"%s\",%u,%u,%u,%u\r\n", ACCESS_POINT_NAME, ACCESS_POINT_PASSWORD, 1, 3, 2, 0); break;  
            case 5: sprintf(cmd, "AT+CIPMODE=%u\r\n", 0); break;
            case 6: sprintf(cmd, "AT+CIPMUX=%u\r\n", 1); break;
            case 7: sprintf(cmd, "AT+CIPSERVER=1,%u\r\n", TCP_SERVER_PORT); break;
            case 8: sprintf(cmd, "AT+CIPSTO=%u\r\n", TCP_SERVER_TIMEOUT); break;       
        }
        wifi_send(cmd);       
        wifi_read();   
        while (wifi_state != DATA_BLOCK_COMPLETE);
        wifi_data_parser();
       
        if (!wifi_cmd_result) {
            wifi_error = true;
            Error_Handler();
        }
 
        HAL_Delay(100);
    }
    free(cmd);
}

void wifi_reset(void) {
    SET_BIT(GPIOH->BSRR, GPIO_BSRR_BR_13);
    HAL_Delay(100);
    SET_BIT(GPIOH->BSRR, GPIO_BSRR_BS_13);    
}

//-------------------------------------------------------------------------
// WIFI auxiliary routines
//-------------------------------------------------------------------------
void wifi_state_checker(void) {
    char symbol = *(wifi_rx_buffer + wifi_parser_position);
    while (symbol || wifi_state != DATA_BLOCK_COMPLETE) {
        switch (wifi_state) {
            case START:
                if (isdigit(symbol)) wifi_state = TCP_INFO_BEGIN;
                else if (symbol == '>') wifi_state = SEND_PROMPT;
                else if (symbol == '\r') wifi_state = STATUS_BEGIN;
                else if (isprint(symbol)) wifi_state = CHAR_RECEIVED;
                break;
            case SEND_PROMPT:
                if (symbol == ' ') wifi_state = DATA_BLOCK_COMPLETE;
                break;
            case CHAR_RECEIVED:
                if (symbol == '\r') wifi_state = WAIT_INFO_LF;
                break;
            case WAIT_INFO_LF:
                if (symbol == '\r') wifi_state = WAIT_ECHO_LF;
                else if (symbol == '\n') wifi_state = INFO_RDY;
                break;
            case WAIT_ECHO_LF:
                if (symbol == '\n') wifi_state = ECHO_RDY;
                break;
            case STATUS_BEGIN:
                if (symbol == '+') wifi_state = MSG_BEGIN;
                else if (symbol == '\r') wifi_state = WAIT_LF;
                break;
            case WAIT_LF:
                if (symbol == '\n') wifi_state = DATA_BLOCK_COMPLETE;
                break;
            case ECHO_RDY: case INFO_RDY:
                if (isprint(symbol)) wifi_state = CHAR_RECEIVED;
                else if (symbol == '\r') wifi_state = STATUS_BEGIN;
                break;
            case MSG_BEGIN: case TCP_INFO_BEGIN:
                if (symbol == '\r') wifi_state = WAIT_LF;
                break;
            default: break;
        }
        ++wifi_parser_position;
        symbol = *(wifi_rx_buffer + wifi_parser_position);
    }
    if (wifi_state == DATA_BLOCK_COMPLETE) HAL_UART_DMAStop(wifi_port);
}

void wifi_data_parser(void) {
    char symbol;
    memset((char*)wifi_response, 0x00, WIFI_BUFFER_SIZE);
        
    wifi_parser_position = 0;
    int current_position = 0;
    
    while ((symbol = *(wifi_rx_buffer + wifi_parser_position)) != '\0') {
        switch (symbol) {
            case '\r': case '\n': 
                ++wifi_parser_position; 
                break;
            case '+': 
                while (*(wifi_rx_buffer + wifi_parser_position++) != ':');
                while ((symbol = *(wifi_rx_buffer + wifi_parser_position++)) != '\r') *(wifi_response + current_position++) = symbol;
                wifi_cmd_result = true;
                goto parser_ready;
            case '1': case '2': case '3': case '4': case '5': case '6': case '7': case '8': case '9': case '0':
                while ((symbol = *(wifi_rx_buffer + wifi_parser_position++)) != '\r') *(wifi_response + current_position++) = symbol;
                wifi_cmd_result = true;
                goto parser_ready;
            case '>':
                strcpy((char *)wifi_response, "TCP RTS");
                wifi_cmd_result = true;
                goto parser_ready;
            case 'R': case 'A':
                while (*(wifi_rx_buffer + wifi_parser_position++) != '\r');
                break;
            default:
                while ((symbol = *(wifi_rx_buffer + wifi_parser_position++)) != '\r') *(wifi_response + current_position++) = symbol;
                if (strstr((char *)wifi_response, "OK") || strstr((char *)wifi_response, "SEND OK")) wifi_cmd_result = true;
                else if (strstr((char *)wifi_response, "ERROR") || strstr((char *)wifi_response, "SEND FAIL")) wifi_cmd_result = false; 
                goto parser_ready;
        }
    }  
    parser_ready: return;
}

void wifi_send(char* message) {
    memset(wifi_tx_buffer, '\0', WIFI_BUFFER_SIZE);
    strcpy(wifi_tx_buffer, message);
    HAL_UART_Transmit_DMA(wifi_port, (uint8_t *)wifi_tx_buffer, strlen(wifi_tx_buffer));
}

void wifi_read(void) {
    memset(wifi_rx_buffer, '\0', WIFI_BUFFER_SIZE);
    wifi_state = START;
    HAL_UART_Receive_DMA(wifi_port, (uint8_t *)wifi_rx_buffer, WIFI_BUFFER_SIZE);
}

void wifi_send_tcp(char* message) {
    if (wifi_tcp_connect_id == 100) return;
    char command[WIFI_BUFFER_SIZE];
    strcpy(wifi_tx_buffer, message);    
    sprintf(command, "AT+CIPSEND=%u,%u\r\n", wifi_tcp_connect_id, strlen(message));
    
    for (int i = 0; i < TCP_SEND_TRESHOLD; ++i) {
        wifi_send(command);
        while (!strstr((char *)wifi_response, "TCP RTS"));
        HAL_UART_Transmit_DMA(wifi_port, (uint8_t *)wifi_tx_buffer, strlen(wifi_tx_buffer));
        
        while (wifi_state != DATA_BLOCK_COMPLETE && !strstr((char *)wifi_response, "SEND"));
        if (wifi_cmd_result) break;
    }
    
    if (!wifi_cmd_result) {
        tcp_error = true;
        Error_Handler();
    }
}

//-------------------------------------------------------------------------
// WIFI information routines
//-------------------------------------------------------------------------
enum receiver_state wifi_get_state(void) {
    return wifi_state;
}

int wifi_get_tcp_connection(void) {
    return wifi_tcp_connect_id;
}

bool wifi_get_cmd_result(void) {
    return wifi_cmd_result;
}

char* wifi_get_command(void) {
    return (char *)wifi_response;
}


