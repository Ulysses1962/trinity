#include "main.h"
#include "usb_device.h"

#pragma once

uint8_t* console_read(void);
void console_write(char* message);
bool console_is_data_ready(void);
