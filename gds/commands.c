#include "commands.h"
#include "serial_num.h"
#include "subbus.h"
#ifdef TIMED_COMMANDS
#include "rtc_timer.h"
#endif

static void commands_init(void) {
	// GPIO on PA04
	gpio_set_pin_level(INV_ARM, false);
	gpio_set_pin_direction(INV_ARM, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(INV_ARM, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA06
	gpio_set_pin_level(CKT3_EN, false);
	gpio_set_pin_direction(CKT3_EN, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CKT3_EN, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA14
	gpio_set_pin_level(CO2_EXH, false);
	gpio_set_pin_direction(CO2_EXH, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CO2_EXH, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA18
	gpio_set_pin_level(CO2_REF, false);
	gpio_set_pin_direction(CO2_REF, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CO2_REF, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA19
	gpio_set_pin_level(CAL_REF, false);
	gpio_set_pin_direction(CAL_REF, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CAL_REF, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA20
	gpio_set_pin_level(CAL_LO, false);
	gpio_set_pin_direction(CAL_LO, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CAL_LO, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PA21
	gpio_set_pin_level(CAL_HI, false);
	gpio_set_pin_direction(CAL_HI, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CAL_HI, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB16
	gpio_set_pin_level(CO2_PUMP, false);
	gpio_set_pin_direction(CO2_PUMP, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CO2_PUMP, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB17
	gpio_set_pin_level(MM_EXH, false);
	gpio_set_pin_direction(MM_EXH, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(MM_EXH, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB22
	gpio_set_pin_level(MM_PUMP, false);
	gpio_set_pin_direction(MM_PUMP, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(MM_PUMP, GPIO_PIN_FUNCTION_OFF);

	// GPIO on PB23
	gpio_set_pin_level(CAL_SPR, false);
	gpio_set_pin_direction(CAL_SPR, GPIO_DIRECTION_OUT);
	gpio_set_pin_function(CAL_SPR, GPIO_PIN_FUNCTION_OFF);

}

static void update_status(uint16_t *status, uint8_t pin, uint16_t bit) {
  if (gpio_get_pin_level(pin)) {
    *status |= bit;
  } else {
    *status &= ~bit;
  }
}

/** Command Status Register Bit mapping 
 *  Bit Signal 
 *   0  CAL_HI
 *   1  CAL_LO
 *   2  CAL_REF
 *   3  CO2_REF
 *   4  CAL_SPR
 *   5  MM_PUMP
 *   6  MM_EXH
 *   7  CO2_PUMP
 *   8  CO2_EXH
 *   9  CKT3_EN
 *  10  INV_ARM
 *  11  
 *  ... unused
 *  15  
 */
static subbus_cache_word_t cmd_cache[CMD_HIGH_ADDR-CMD_BASE_ADDR+1] = {
  { 0, 0, true,  false, true, false, false } // Offset 0: R: ADC Flow 0
};

static uint8_t cmd_pins[N_CMD_PINS] = { CAL_HI, CAL_LO, CAL_REF, 
CO2_REF, CAL_SPR, MM_PUMP, MM_EXH, CO2_PUMP, CO2_EXH, CKT3_EN, INV_ARM
}; 

#ifdef TIMED_COMMANDS

typedef struct {
  int when;
  uint16_t cmd;
} timed_cmd_t;

static timed_cmd_t timed_cmds[] = TIMED_COMMANDS;
#define N_TIMED_CMDS (sizeof(timed_cmds)/sizeof(timed_cmd_t))
static int timed_cmds_executed = 0;

#endif

static void cmd_poll(void) {
  uint16_t cmd;
  uint16_t status;

#ifdef N_TIMED_CMDS
  bool have_cmd = false;
  if (timed_cmds_executed < N_TIMED_CMDS && rtc_current_count >= timed_cmds[timed_cmds_executed].when) {
    cmd = timed_cmds[timed_cmds_executed++].cmd;
    have_cmd = true;
  } else if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
    have_cmd = true;
  }
  if (have_cmd) {
#else
  if (subbus_cache_iswritten(&sb_cmd, CMD_BASE_ADDR, &cmd)) {
#endif

    if (cmd/2 < N_CMD_PINS) {
      uint8_t pin = cmd_pins[cmd/2];
      gpio_set_pin_level(pin, cmd & 1);
    }
  }

  status = 0;
  update_status(&status, CAL_HI, 0x0001);
  update_status(&status, CAL_LO, 0x0002);
  update_status(&status, CAL_REF, 0x0004);
  update_status(&status, CO2_REF, 0x0008);
  update_status(&status, CAL_SPR, 0x0010);
  update_status(&status, MM_PUMP, 0x0020);
  update_status(&status, MM_EXH, 0x0040);
  update_status(&status, CO2_PUMP, 0x0080);
  update_status(&status, CO2_EXH, 0x0100);
  update_status(&status, CKT3_EN, 0x0200);
  update_status(&status, INV_ARM, 0x0400);
  sb_cache_update(cmd_cache, 0, status); // Make status bits true in high
}

static void cmd_reset(void) {
  commands_init();
  if (!sb_cmd.initialized) {
    sb_cmd.initialized = true;
  }
}

subbus_driver_t sb_cmd = {
  CMD_BASE_ADDR, CMD_HIGH_ADDR, // address range
  cmd_cache,
  cmd_reset,
  cmd_poll,
  false
};
