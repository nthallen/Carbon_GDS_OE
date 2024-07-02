#ifndef COMMANDS_H_INCLUDED
#define COMMANDS_H_INCLUDED

#include "gds_pins.h"
#include "subbus.h"

#define N_CMD_PINS 11
#define CMD_BASE_ADDR 0x10
#define CMD_HIGH_ADDR 0x10

extern subbus_driver_t sb_cmd;

#endif
