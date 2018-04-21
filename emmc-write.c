/*
 * Copyright (C) 2016  Siarhei Siamashka <siarhei.siamashka@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Partially based on the uart code from ar100-info
 *
 * (C) Copyright 2013 Stefan Kristiansson <stefan.kristiansson@saunalahti.fi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Partially based on the sunxi gpio code from U-Boot
 *
 * (C) Copyright 2012 Henrik Nordstrom <henrik@henriknordstrom.net>
 *
 * Based on earlier arch/arm/cpu/armv7/sunxi/gpio.c:
 *
 * (C) Copyright 2007-2011
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Tom Cubie <tangliang@allwinnertech.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

typedef unsigned int u32;

#define set_wbit(addr, v)	(*((volatile unsigned long  *)(addr)) |= (unsigned long)(v))
#define readl(addr)		(*((volatile unsigned long  *)(addr)))
#define writel(v, addr)		(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))

#define H6_UART0_BASE		0x05000000
#define H6_PIO_BASE		0x0300B000
#define H6_R_PIO_BASE		0x07022000
#define H6_CCM_BASE		0x03001000
#define H6_SRAMCTRL_BASE	0x03000000

/*****************************************************************************
 * GPIO code, borrowed from U-Boot                                           *
 *****************************************************************************/

#define SUNXI_GPIO_A    0
#define SUNXI_GPIO_B    1
#define SUNXI_GPIO_C    2
#define SUNXI_GPIO_D    3
#define SUNXI_GPIO_E    4
#define SUNXI_GPIO_F    5
#define SUNXI_GPIO_G    6
#define SUNXI_GPIO_H    7
#define SUNXI_GPIO_I    8
#define SUNXI_GPIO_J    9
#define SUNXI_GPIO_K    10

#define SUNXI_GPIO_L    0
#define SUNXI_GPIO_M    1

struct sunxi_gpio {
	u32 cfg[4];
	u32 dat;
	u32 drv[2];
	u32 pull[2];
};

struct sunxi_gpio_reg {
	struct sunxi_gpio gpio_bank[10];
};

#define GPIO_BANK(pin)		((pin) >> 5)
#define GPIO_NUM(pin)		((pin) & 0x1F)

#define GPIO_CFG_INDEX(pin)	(((pin) & 0x1F) >> 3)
#define GPIO_CFG_OFFSET(pin)	((((pin) & 0x1F) & 0x7) << 2)

#define GPIO_PULL_INDEX(pin)	(((pin) & 0x1f) >> 4)
#define GPIO_PULL_OFFSET(pin)	((((pin) & 0x1f) & 0xf) << 1)

/* GPIO bank sizes */
#define SUNXI_GPIO_A_NR    (32)
#define SUNXI_GPIO_B_NR    (32)
#define SUNXI_GPIO_C_NR    (32)
#define SUNXI_GPIO_D_NR    (32)
#define SUNXI_GPIO_E_NR    (32)
#define SUNXI_GPIO_F_NR    (32)
#define SUNXI_GPIO_G_NR    (32)
#define SUNXI_GPIO_H_NR    (32)
#define SUNXI_GPIO_I_NR    (32)
#define SUNXI_GPIO_L_NR    (32)
#define SUNXI_GPIO_M_NR    (32)

#define SUNXI_GPIO_NEXT(__gpio) ((__gpio##_START) + (__gpio##_NR) + 0)

enum sunxi_gpio_number {
	SUNXI_GPIO_A_START = 0,
	SUNXI_GPIO_B_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_A),
	SUNXI_GPIO_C_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_B),
	SUNXI_GPIO_D_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_C),
	SUNXI_GPIO_E_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_D),
	SUNXI_GPIO_F_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_E),
	SUNXI_GPIO_G_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_F),
	SUNXI_GPIO_H_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_G),
	SUNXI_GPIO_I_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_H),
};
enum sunxi_r_gpio_number {
	SUNXI_GPIO_L_START = 0,
	SUNXI_GPIO_M_START = SUNXI_GPIO_NEXT(SUNXI_GPIO_L),
};

/* SUNXI GPIO number definitions */
#define SUNXI_GPA(_nr)          (SUNXI_GPIO_A_START + (_nr))
#define SUNXI_GPB(_nr)          (SUNXI_GPIO_B_START + (_nr))
#define SUNXI_GPC(_nr)          (SUNXI_GPIO_C_START + (_nr))
#define SUNXI_GPD(_nr)          (SUNXI_GPIO_D_START + (_nr))
#define SUNXI_GPE(_nr)          (SUNXI_GPIO_E_START + (_nr))
#define SUNXI_GPF(_nr)          (SUNXI_GPIO_F_START + (_nr))
#define SUNXI_GPG(_nr)          (SUNXI_GPIO_G_START + (_nr))
#define SUNXI_GPH(_nr)          (SUNXI_GPIO_H_START + (_nr))
#define SUNXI_GPI(_nr)          (SUNXI_GPIO_I_START + (_nr))
#define SUNXI_GPL(_nr)          (SUNXI_GPIO_L_START + (_nr))
#define SUNXI_GPM(_nr)          (SUNXI_GPIO_M_START + (_nr))

/* GPIO pin function config */
#define SUNXI_GPIO_INPUT        (0)
#define SUNXI_GPIO_OUTPUT       (1)
#define SUN4I_GPB_UART0         (2)
#define SUN5I_GPB_UART0         (2)
#define SUN6I_GPH_UART0         (2)
#define SUN8I_H3_GPA_UART0      (2)
#define SUN8I_V3S_GPB_UART0	(3)
#define SUN50I_H5_GPA_UART0     (2)
#define SUN50I_H6_GPH_UART0	(2)
#define SUN50I_A64_GPB_UART0    (4)
#define SUNXI_GPF_UART0         (4)

/* GPIO pin pull-up/down config */
#define SUNXI_GPIO_PULL_DISABLE (0)
#define SUNXI_GPIO_PULL_UP      (1)
#define SUNXI_GPIO_PULL_DOWN    (2)

static u32 pio_base, r_pio_base;

int sunxi_gpio_set_cfgpin(u32 pin, u32 val)
{
	u32 cfg;
	u32 bank = GPIO_BANK(pin);
	u32 index = GPIO_CFG_INDEX(pin);
	u32 offset = GPIO_CFG_OFFSET(pin);
	struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *)pio_base)->gpio_bank[bank];
	cfg = readl(&pio->cfg[0] + index);
	cfg &= ~(0xf << offset);
	cfg |= val << offset;
	writel(cfg, &pio->cfg[0] + index);
	return 0;
}

int sunxi_gpio_set_pull(u32 pin, u32 val)
{
	u32 cfg;
	u32 bank = GPIO_BANK(pin);
	u32 index = GPIO_PULL_INDEX(pin);
	u32 offset = GPIO_PULL_OFFSET(pin);
	struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *)pio_base)->gpio_bank[bank];
	cfg = readl(&pio->pull[0] + index);
	cfg &= ~(0x3 << offset);
	cfg |= val << offset;
	writel(cfg, &pio->pull[0] + index);
	return 0;
}

int sunxi_gpio_output(u32 pin, u32 val)
{
	u32 dat;
	u32 bank = GPIO_BANK(pin);
	u32 num = GPIO_NUM(pin);
	struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *)pio_base)->gpio_bank[bank];
	dat = readl(&pio->dat);
	if(val)
		dat |= 1 << num;
	else
		dat &= ~(1 << num);
	writel(dat, &pio->dat);
	return 0;
}

int sunxi_gpio_input(u32 pin)
{
	u32 dat;
	u32 bank = GPIO_BANK(pin);
	u32 num = GPIO_NUM(pin);
	struct sunxi_gpio *pio = &((struct sunxi_gpio_reg *)pio_base)->gpio_bank[bank];
	dat = readl(&pio->dat);
	dat >>= num;
	return (dat & 0x1);
}

int gpio_direction_input(unsigned gpio)
{
	sunxi_gpio_set_cfgpin(gpio, SUNXI_GPIO_INPUT);
	return sunxi_gpio_input(gpio);
}

int gpio_direction_output(unsigned gpio, int value)
{
	sunxi_gpio_set_cfgpin(gpio, SUNXI_GPIO_OUTPUT);
	return sunxi_gpio_output(gpio, value);
}

/*****************************************************************************
 * Nearly all the Allwinner SoCs are using the same VER_REG register for     *
 * runtime SoC type identification. For additional details see:              *
 *                                                                           *
 *    https://linux-sunxi.org/SRAM_Controller_Register_Guide                 *
 *                                                                           *
 * Allwinner A80 is an oddball and has a non-standard address of the VER_REG *
 *                                                                           *
 * Allwinner A10s and A13 are using the same SoC type id, but they can be    *
 * differentiated using a certain part of the SID register.                  *
 *                                                                           *
 * Allwinner H6 has its memory map totally reworked, but the SRAM controller *
 * remains similar; the base of it is moved to 0x03000000.                   *
 *****************************************************************************/

#define VER_REG			(AW_SRAMCTRL_BASE + 0x24)
#define H6_VER_REG		(H6_SRAMCTRL_BASE + 0x24)
#define SUN4I_SID_BASE		0x01C23800
#define SUN8I_SID_BASE		0x01C14000

#define SID_PRCTL	0x40	/* SID program/read control register */
#define SID_RDKEY	0x60	/* SID read key value register */

#define SID_OP_LOCK	0xAC	/* Efuse operation lock value */
#define SID_READ_START	(1 << 1) /* bit 1 of SID_PRCTL, Software Read Start */

u32 sid_read_key(u32 sid_base, u32 offset)
{
	u32 reg_val;

	reg_val = (offset & 0x1FF) << 16; /* PG_INDEX value */
	reg_val |= (SID_OP_LOCK << 8) | SID_READ_START; /* request read access */
	writel(reg_val, sid_base + SID_PRCTL);

	while (readl(sid_base + SID_PRCTL) & SID_READ_START) ; /* wait while busy */

	reg_val = readl(sid_base + SID_RDKEY); /* read SID key value */
	writel(0, sid_base + SID_PRCTL); /* clear SID_PRCTL (removing SID_OP_LOCK) */

	return reg_val;
}

/*****************************************************************************
 * UART is mostly the same on A10/A13/A20/A31/H3/A64, except that newer SoCs *
 * have changed the APB numbering scheme (A10/A13/A20 used to have APB0 and  *
 * APB1 names, but newer SoCs just have renamed them into APB1 and APB2).    *
 * The constants below are using the new APB numbering convention.           *
 * Also the newer SoCs have introduced the APB2_RESET register, but writing  *
 * to it effectively goes nowhere on older SoCs and is harmless.             *
 *****************************************************************************/

#define CONFIG_CONS_INDEX	1
#define H6_UART_GATE_RESET	(H6_CCM_BASE + 0x90C)
#define H6_UART_GATE_SHIFT	(0)
#define H6_UART_RESET_SHIFT	(16)

void clock_init_uart_h6(void)
{
	/* Open the clock gate for UART0 */
	set_wbit(H6_UART_GATE_RESET, 1 << (H6_UART_GATE_SHIFT + CONFIG_CONS_INDEX - 1));
	/* Deassert UART0 reset */
	set_wbit(H6_UART_GATE_RESET, 1 << (H6_UART_RESET_SHIFT + CONFIG_CONS_INDEX - 1));
}

void clock_init_uart(void)
{
	clock_init_uart_h6();
}

void delay_us(unsigned n)
{
	int i = n;
	while(i--) {
		n++;
	}
}

static void led_switch(int on_off)
{
	unsigned int value = *(unsigned int *)(r_pio_base + 0x10);
	unsigned int red = value & 0x00000010;
	unsigned int green = value & 0x00000080;
	if((!red && !green) || on_off)
	{
		value |= 0x00000010 | 0x00000080;
	} else if(red) {
		value &= ~ red;
		value |= 0x00000080;
	} else {
		value &= ~ green;
		value |= 0x00000010;
	}
	/*默认开启PER_LED*/
	*(unsigned int *)(r_pio_base + 0x10) = value;
}

/*****************************************************************************
 * UART0 pins muxing is different for different SoC variants.                *
 * Allwinner A13 is a bit special, because there are no dedicated UART0 pins *
 * and they are shared with MMC0.                                            *
 *****************************************************************************/

void gpio_init(void)
{
	/*PL4,PL7寄存器配置为输出模式*/
	{
		*(unsigned int *)r_pio_base |= 0x70070000; //16, 28
		*(unsigned int *)r_pio_base &= 0x1FF1FFFF; //16, 28
	}
	sunxi_gpio_set_cfgpin(SUNXI_GPH(0), SUN50I_H6_GPH_UART0);
	sunxi_gpio_set_cfgpin(SUNXI_GPH(1), SUN50I_H6_GPH_UART0);
	sunxi_gpio_set_pull(SUNXI_GPH(1), SUNXI_GPIO_PULL_UP);
}

/*****************************************************************************/

static u32 uart0_base;

#define UART0_RBR (uart0_base + 0x0)    /* receive buffer register */
#define UART0_THR (uart0_base + 0x0)    /* transmit holding register */
#define UART0_DLL (uart0_base + 0x0)    /* divisor latch low register */

#define UART0_DLH (uart0_base + 0x4)    /* divisor latch high register */
#define UART0_IER (uart0_base + 0x4)    /* interrupt enable reigster */

#define UART0_IIR (uart0_base + 0x8)    /* interrupt identity register */
#define UART0_FCR (uart0_base + 0x8)    /* fifo control register */

#define UART0_LCR (uart0_base + 0xc)    /* line control register */

#define UART0_LSR (uart0_base + 0x14)   /* line status register */

#define BAUD_115200    (0xD) /* 24 * 1000 * 1000 / 16 / 115200 = 13 */
#define NO_PARITY      (0)
#define ONE_STOP_BIT   (0)
#define DAT_LEN_8_BITS (3)
#define LC_8_N_1       (NO_PARITY << 3 | ONE_STOP_BIT << 2 | DAT_LEN_8_BITS)

void uart0_init(void)
{
	clock_init_uart();

	/* select dll dlh */
	writel(0x80, UART0_LCR);
	/* set baudrate */
	writel(0, UART0_DLH);
	writel(BAUD_115200, UART0_DLL);
	/* set line control */
	writel(LC_8_N_1, UART0_LCR);
}

void uart0_putc(char c)
{
	while (!(readl(UART0_LSR) & (1 << 6))) {}
	writel(c, UART0_THR);
}

void uart0_puts(const char *s)
{
	while (*s) {
		if (*s == '\n')
			uart0_putc('\r');
		uart0_putc(*s++);
	}
}

/*****************************************************************************/

/* A workaround for https://patchwork.ozlabs.org/patch/622173 */
void __attribute__((section(".start"))) __attribute__((naked)) start(void)
{
	asm volatile("b     main             \n"
		     ".long 0xffffffff       \n"
		     ".long 0xffffffff       \n"
		     ".long 0xffffffff       \n");
}

enum { BOOT_DEVICE_UNK, BOOT_DEVICE_FEL, BOOT_DEVICE_MMC0, BOOT_DEVICE_SPI };

void bases_init(void)
{
	pio_base = H6_PIO_BASE;
	r_pio_base = H6_R_PIO_BASE;
	uart0_base = H6_UART0_BASE;
}

int write_image_to_emmc(void)
{
	u32 *spl_signature;
	spl_signature = (void *)0x00020004;

	uart0_puts((void*)spl_signature);

	/* Check the eGON.BT0 magic in the SPL header */
	if (spl_signature[0] != 0x4E4F4765 || spl_signature[1] != 0x3054422E)
		return BOOT_DEVICE_FEL;

	u32 boot_dev = spl_signature[9] & 0xFF; /* offset into SPL = 0x28 */
	if (boot_dev == 0)
		return BOOT_DEVICE_MMC0;
	if (boot_dev == 3)
		return BOOT_DEVICE_SPI;

	uart0_puts("unknown boot devices.\n");
	return BOOT_DEVICE_UNK;
}

int main(void)
{
	bases_init();
	gpio_init();
	uart0_init();

	led_switch(0);
	uart0_puts("\nHello from Allwinner H6!\n");

	/* TODO: wirte image from memory to mmc */
	write_image_to_emmc();

	return 0;
}
