/*
 * This file was based on code originally generated from Atmel START in main.c, though
 * it has been heavily modified. Nevertheless, whenever the Atmel START project is
 * updated, any changes to main.c (which is no longer compiled into this project)
 * should be reviewed to determine whether they should be propagated into the
 * project.
 */

#include <hal_init.h>
#include "subbus.h"
#include "ser_control.h"
#include "gds_usb.h"
#include "commands.h"
#include "i2c.h"
#include "rtc_timer.h"

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	init_mcu();

  if (subbus_add_driver(&sb_base)
   || subbus_add_driver(&sb_fail_sw)
   || subbus_add_driver(&sb_board_desc)
   || subbus_add_driver(&sb_control)
   || subbus_add_driver(&sb_usb)
   || subbus_add_driver(&sb_cmd)
   || subbus_add_driver(&sb_i2c_adc)
   || subbus_add_driver(&sb_i2c_ms)
   || subbus_add_driver(&sb_rtc)
  ) {
    while (true) ; // some driver is misconfigured.
  }
  subbus_reset(); // Resets all drivers
  while (1) {
    subbus_poll();
  }
}
