#include <stdint.h>
#include "resource_table_empty.h"
#include <pru_cfg.h>

#define PRU0
/*#define PRU1*/

#define BIT(x) (1 << x)

#ifdef PRU0
	#define CLK		BIT(0) /*P9_31 IN*/
	#define CS		BIT(3) /*P9_28 IN*/
	#define MOSI		BIT(1) /*P9_29 IN*/
	#define MISO		BIT(2) /*P9_30 OUT*/
#endif

#ifdef PRU1
	#define CLK		BIT(6) /*P8_39 IN*/
	#define CS		BIT(7) /*P8_40 IN*/
	#define MOSI		BIT(4) /*P8_41 IN*/
	#define MISO		BIT(5) /*P8_42 OUT*/
#endif

#define PRU_MEM_OFFSET		0x100000
#define SLAVE_STATUS		0x00
#define SLAVE_CONFIG		0x04

#define SLAVE_STATUS_READY	BIT(0)
#define SLAVE_STATUS_EOT	BIT(1)

#define SLAVE_CONFIG_RESET	BIT(0)
#define SLAVE_CONFIG_CS		BIT(1)
#define SLAVE_CONFIG_CS_POL	BIT(2)
#define SLAVE_CONFIG_CPOL	BIT(3)
#define SLAVE_CONFIG_CPHA	BIT(4)
#define SLAVE_MODE		(0x03 << 5)
#define SLAVE_TM		BIT(5)
#define SLAVE_RM		BIT(6)
#define SLAVE_BITS_PER_WORD	(0x1F << 7)

register uint32_t __R30;
register uint32_t __R31;

void main(void)
{
	uint32_t *config_reg = (uint32_t)(PRU_MEM_OFFSET + SLAVE_CONFIG);
	uint32_t *status_reg = (uint32_t)(PRU_MEM_OFFSET + SLAVE_STATUS);

	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	while (1) {
		while (!(__R31 & CS))
		__R30 |= MISO;
		__delay_cycles(100);
		while ((__R31 & CS))
		__R30 &= ~MISO;
		__delay_cycles(100);

	}
}
