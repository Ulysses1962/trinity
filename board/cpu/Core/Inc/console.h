#include "main.h"

#ifndef __CONSOLE_H_
#define __CONSOLE_H_

void console_write(char* message);
bool console_is_data_ready(void);
void reset_data_ready(void);
void console_read(char* message);

#endif


