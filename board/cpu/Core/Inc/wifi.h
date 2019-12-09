#include "main.h"
#include "stm32f7xx_hal.h"
#include "ctype.h"

#ifndef __WIFI_H_
#define __WIFI_H_

extern char wifi_rx_buffer[WIFI_BUFFER_SIZE];
extern char wifi_tx_buffer[WIFI_BUFFER_SIZE];
extern UART_HandleTypeDef huart3;

extern bool wifi_error;
extern bool tcp_error;
//-------------------------------------------------------------------------
// WIFI initialization routines
//-------------------------------------------------------------------------
void wifi_init(void);
void wifi_reset(void);

//-------------------------------------------------------------------------
// WIFI information routines
//-------------------------------------------------------------------------
enum  receiver_state wifi_get_state(void);
int   wifi_get_tcp_connection(void);
bool  wifi_get_cmd_result(void);
char* wifi_get_command(void);

//-------------------------------------------------------------------------
// WIFI auxiliary routines
//-------------------------------------------------------------------------
void wifi_state_checker(void);
void wifi_data_parser(void);
void wifi_send(char* message);
void wifi_read(void);
void wifi_send_tcp(char* message);

#endif


