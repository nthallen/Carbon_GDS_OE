#ifndef SUBBUS_H_INCLUDED
#define SUBBUS_H_INCLUDED

#include <stdint.h>
#include <stdbool.h>
//#include "serial_num.h"

#define USE_SUBBUS 1

#if USE_SUBBUS
#define SUBBUS_FAIL_RESERVED        0xF000
#define SUBBUS_INTA_ADDR            0x0001
#define SUBBUS_BDID_ADDR            0x0002
#define SUBBUS_BLDNO_ADDR           0x0003
#define SUBBUS_BDSN_ADDR            0x0004
#define SUBBUS_INSTID_ADDR          0x0005
#define SUBBUS_FAIL_ADDR            0x0006
#define SUBBUS_SWITCHES_ADDR        0x0007
#define SUBBUS_DESC_FIFO_SIZE_ADDR  0x0008
#define SUBBUS_DESC_FIFO_ADDR       0x0009
#define SUBBUS_SUBFUNCTION_ADDR     0x000A
#define SUBBUS_INTERRUPTS           0

#if SUBBUS_INTERRUPTS
extern volatile uint8_t subbus_intr_req;
void init_interrupts(void);
int intr_attach(int id, uint16_t addr);
int intr_detach( uint16_t addr );
void intr_service(void);
#endif
int subbus_read( uint16_t addr, uint16_t *rv );
int subbus_write( uint16_t addr, uint16_t data);
void subbus_reset(void);
void subbus_poll(void);
void set_fail(uint16_t arg);

typedef struct {
  /** The current value of this word */
  uint16_t cache;
  /** The value that has been written. Allows the driver code to do checks for validity */
  uint16_t wvalue;
  /** True if this word is readable */
  bool readable;
  /** True if this word has been read */
  bool was_read;
  /** True if this word is writable */
  bool writable;
  /** True if this word has been written */
  bool written;
  /** True to invoke sb_action immediately rather than waiting for poll */
  bool dynamic;
} subbus_cache_word_t;

typedef struct subbus_driver_s {
  uint16_t low, high;
  subbus_cache_word_t *cache;
  void (*reset)(void);
  void (*poll)(void);
  void (*sb_action)(uint16_t offset); // called if dynamic
  bool initialized;
  struct subbus_driver_s *next;
} subbus_driver_t;

bool subbus_add_driver(subbus_driver_t *driver);
extern subbus_driver_t sb_base;
extern subbus_driver_t sb_fail_sw;
extern subbus_driver_t sb_board_desc;

/**
 * Fully qualified check that the specified subbus address is both
 * valid and has been written. If so, copies the written value into
 * the value argument and resets the iswritten flag.
 * @param drv pointer to the subbus_driver_t struct
 * @param addr the subbus address
 * @param value pointer where the new value should be stored
 * @return true if the address is valid and the iswritten flag was set
 */
bool subbus_cache_iswritten(subbus_driver_t *drv, uint16_t addr, uint16_t *value);

/**
 * Unqualified check of the local cache similar to subbus_cache_iswritten, but
 * using the offset within the local cache instead of the global address.
 * @param cache pointer to the local cache
 * @param offset of the register in the local cache
 * @param value pointer where the new value should be stored
 * @return true if the iswritten flag was set
 */
bool sb_cache_iswritten(subbus_cache_word_t *cache, uint16_t offset, uint16_t *value);
bool subbus_cache_was_read(subbus_driver_t *drv, uint16_t addr);
bool sb_cache_was_read(subbus_cache_word_t *cache, uint16_t offset);

/**
 * This function differs from subbus_write() in that it directly
 * updates the cache value. subbus_write() is specifically for
 * write originating from the control port. subbus_cache_update() is
 * used by internal functions for storing data acquired from
 * peripherals, or for storing values written from the control
 * port after verifying them.
 * @param drv The driver structure
 * @param addr The cache address
 * @param data The value to be written
 * @return true on success
 */
bool subbus_cache_update(subbus_driver_t *drv, uint16_t addr, uint16_t data);
/**
 * This function differs from subbus_write() in that it directly
 * updates the cache value.
 * This function differes from subbus_cache_update() in that it
 * directly addresses the local cache without checking the range.
 * On success, returns true and clears the was_read flag.
 * @param cache pointer to the local cache
 * @param offset of the register in the local cache
 * @param data The value to be written
 * @return true on success
 */
bool sb_cache_update(subbus_cache_word_t *cache, uint16_t offset, uint16_t data);
/**
 * This function differs from subbus_write() in that it directly
 * updates the cache value.
 * This function differes from subbus_cache_update() in that it
 * directly addresses the local cache without checking the range.
 * On success, returns true and clears the was_read flag.
 * This function differs from both in that it updates two successive
 * addresses, placing the low-order 16 bits in the lower register
 * and the high-order 16 bits in the following register.
 * @param cache pointer to the local cache
 * @param offset of the register in the local cache
 * @param data The value to be written
 * @return true on success
 */
bool sb_cache_update32(subbus_cache_word_t *cache, uint16_t offset, void *data);

/**
 * Indicates that the host computer is actively taking data.
 * Resets the fail timeout clock if present.
 **/
void sb_fail_tick();

#ifndef SB_FAIL_TIMEOUT_SECS
#define SB_FAIL_TIMEOUT_SECS 120
#endif

#endif // USE_SUBBUS

#endif
