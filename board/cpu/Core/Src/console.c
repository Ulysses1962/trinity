#include "console.h"
#include "usbd_cdc_if.h"

extern uint8_t UserRxBufferFS[512];
extern uint8_t UserTxBufferFS[512];
extern __IO bool cdcDataReady;

void console_write(char* message) {
    memset((char*)UserTxBufferFS, '\0', 512);
	memcpy((char*)UserTxBufferFS, message, strlen(message));
	CDC_Transmit_FS((uint8_t*)UserTxBufferFS, strlen((char*)UserTxBufferFS));
}

bool console_is_data_ready(void) {
	return cdcDataReady;
}

void reset_data_ready(void) {
    cdcDataReady = false;
}

void console_read(char* message) {
	  memcpy(message, (char*)UserRxBufferFS, strlen((char*)UserRxBufferFS));
	  memset((char*)UserRxBufferFS, '\0', 512); 
}



