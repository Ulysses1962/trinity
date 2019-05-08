#include "console.h"
#include "usbd_cdc_if.h"

extern uint8_t UserRxBufferFS[1000];
extern uint8_t UserTxBufferFS[1000];
__IO bool cdcDataReady = false;

uint8_t* console_read(void) {
	return UserRxBufferFS;
}

void console_write(char* message) {
    memset((char*)UserTxBufferFS, '\0', 1000);
	memcpy((char*)UserTxBufferFS, message, strlen(message));
	CDC_Transmit_FS((uint8_t*)UserTxBufferFS, strlen((char*)UserTxBufferFS));
}

bool console_is_data_ready(void) {
	return cdcDataReady;
}

