#ifndef SER_CONTROL_H_INCLUDED
#define SER_CONTROL_H_INCLUDED
#include <stdint.h>
#include "subbus.h"

void SendMsg(const char *);			// Send String Back to Host via USB
void SendCode(int8_t code);
void SendCodeVal(int8_t, uint16_t);
void SendErrorMsg(const char *msg);
void poll_control(void);
extern subbus_driver_t sb_control;
#define CONTROL_BASE_ADDR 0x0B
#define CONTROL_HIGH_ADDR 0x0A

#endif