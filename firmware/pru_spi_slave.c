#include <stdint.h>
#include "resource_table_empty.h"
#include <pru_cfg.h>

#define PRU0
/*#define PRU1*/

#ifdef PRU0
	#define CLK		0 /*P9_31 IN*/
	#define CS		3 /*P9_28 IN*/
	#define MOSI		1 /*P9_29 IN*/
	#define MISO		2 /*P9_30 OUT*/
#endif

#ifdef PRU1
	#define CLK		6 /*P8_39 IN*/
	#define CS		7 /*P8_40 IN*/
	#define MOSI		4 /*P8_41 IN*/
	#define MISO		5 /*P8_42 OUT*/
#endif

register uint32_t __R30;
register uint32_t __R31;

void main(void)
{
	uint32_t gpio = 0;

	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
	gpio = 0x000F;

	while (1) {
		__R30 ^= gpio;
		__delay_cycles(100000000);
	}
}
