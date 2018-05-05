/*
 * (C) Copyright 2013-2016
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */
/*
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Description: MMC driver for General mmc operations
 * Author: Aaron <leafy.myeh@allwinnertech.com>
 * Date: 2012-2-3 14:18:18
 */
#include "mmc_def.h"
#include "mmc_bsp.h"
#include "mmc.h"
#include "private_boot0.h"
#include "type.h"
//extern void * memset(void * s, int c, size_t count);
typedef unsigned int u32;

#define set_wbit(addr, v)	(*((volatile unsigned long  *)(addr)) |= (unsigned long)(v))
//#define readl(addr)		(*((volatile unsigned long  *)(addr)))
//#define writel(v, addr)		(*((volatile unsigned long  *)(addr)) = (unsigned long)(v))

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


static int printf(const char *fmt, ...){return 0;}

/* Set block count limit because of 16 bit register limit on some hardware*/
#ifndef CONFIG_SYS_MMC_MAX_BLK_COUNT
#define CONFIG_SYS_MMC_MAX_BLK_COUNT 65535
#endif

//小端

typedef __u32 __be32;
#define __be32_to_cpu(x) ((__u32)(__be32)(x))

static struct mmc* mmc_devices[MAX_MMC_NUM];
static int mmc_config_addr=0;

void __aeabi_llsl(void)
{
#define al r0
#define ah r1
__asm__(
	"cmp	r2, #31\n\r"
	"bhi	1f\n\r"
			
	"movs	r3, r1\n\r"
	"lsls	r0, r2\n\r"
	"lsls	r1, r2\n\r"
	"rsbs	r2, r2, #0\n\r"
	"adds	r2, #32\n\r"
	"lsrs	r3, r2\n\r"
	"orrs	r1, r3\n\r"
	"bx	lr\n\r"

"1:	subs	r2, #32\n\r"
	"movs	r1, r0\n\r"
	"lsls	r1, r2\n\r"
	"movs	r0, #0\n\r"
	"bx	lr\n\r"
       );
}

static void * memcpy(void * dest,const void *src,size_t count)
{
	char *tmp = (char *) dest, *s = (char *) src;
	while (count--)
		*tmp++ = *s++;
	return dest;
}
/*
static void * memset(void * s, int c, size_t count)
{
	char *xs = (char *) s;
	while (count--)
		*xs++ = c;
	return s;
}
*/

static void delay_us(unsigned int n)
{
	unsigned int i = n;
        while(i--)
	{
		 n++;        
	}
 }
static void __msdelay(unsigned int ms)
{
	delay_us(ms*1000);
	return ;
}

int mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd, struct mmc_data *data)
{
	return mmc->send_cmd(mmc, cmd, data);
}

int mmc_send_status(struct mmc *mmc, int timeout)
{
	struct mmc_cmd cmd;
	int err;

	cmd.cmdidx = MMC_CMD_SEND_STATUS;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = 0;

	do {
		err = mmc_send_cmd(mmc, &cmd, NULL);
		if (err){
			mmcinfo("mmc %d Send status failed\n",mmc->control_num);
			return err;
		}
		else if (cmd.response[0] & MMC_STATUS_RDY_FOR_DATA)
			break;

		__msdelay(1);

		if (cmd.response[0] & MMC_STATUS_MASK) {
			mmcinfo("mmc %d Status Error: 0x%08X\n",mmc->control_num, cmd.response[0]);
			return COMM_ERR;
		}
	} while (timeout--);

	if (!timeout) {
		mmcinfo("mmc %d Timeout waiting card ready\n",mmc->control_num);
		return TIMEOUT;
	}

	return 0;
}

int mmc_set_blocklen(struct mmc *mmc, int len)
{
	struct mmc_cmd cmd;

	/* don't set blocklen at ddr mode */
	if ((mmc->speed_mode == HSDDR52_DDR50) || (mmc->speed_mode == HS400)) {
		return 0;
	}

	cmd.cmdidx = MMC_CMD_SET_BLOCKLEN;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = len;
	cmd.flags = 0;

	return mmc_send_cmd(mmc, &cmd, NULL);
}

struct mmc *find_mmc_device(int dev_num)
{
	if (mmc_devices[dev_num] != NULL)
		return mmc_devices[dev_num];

	mmcinfo("MMC Device %d not found\n", dev_num);

	return NULL;
}
#if 0
static unsigned long mmc_erase_t(struct mmc *mmc, unsigned long start, unsigned blkcnt)
{
	struct mmc_cmd cmd;
	unsigned long end;
	int err, start_cmd, end_cmd;

	if (mmc->high_capacity)
		end = start + blkcnt - 1;
	else {
		end = (start + blkcnt - 1) * mmc->write_bl_len;
		start *= mmc->write_bl_len;
	}

	if (IS_SD(mmc)) {
		start_cmd = SD_CMD_ERASE_WR_BLK_START;
		end_cmd = SD_CMD_ERASE_WR_BLK_END;
	} else {
		start_cmd = MMC_CMD_ERASE_GROUP_START;
		end_cmd = MMC_CMD_ERASE_GROUP_END;
	}

	cmd.cmdidx = start_cmd;
	cmd.cmdarg = start;
	cmd.resp_type = MMC_RSP_R1;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		goto err_out;

	cmd.cmdidx = end_cmd;
	cmd.cmdarg = end;

	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		goto err_out;

	cmd.cmdidx = MMC_CMD_ERASE;
	cmd.cmdarg = SECURE_ERASE;
	cmd.resp_type = MMC_RSP_R1b;

	err = mmc_send_cmd(mmc, &cmd, NULL);
	if (err)
		goto err_out;

	return 0;

err_out:
	mmcdbg("mmc erase failed\n");
	return err;
}
#endif
#if 0
int
mmc_berase(int dev_num, unsigned long start, unsigned blkcnt)
{
	int err = 0;
	struct mmc *mmc = find_mmc_device(dev_num);
//	unsigned blk = 0, blk_r = 0;
	void* src = (void*)0x41000000;

	if (!mmc){
		mmcinfo("Can not find mmc dev %d\n",dev_num);
		return -1;
	}

	memset(src, 0, 512*blkcnt);
	mmcinfo("mmc %d erase blk %d ~ %d\n",mmc->control_num, start, start + blkcnt - 1);
	err = mmc_bwrite(dev_num, start, blkcnt, src);
	//mmcinfo("erase flag%d",err);
	if(!err)
	{
		mmcinfo("mmc %d erase failed\n",mmc->control_num);
	}

	return err;
	/*
	if ((start % mmc->erase_grp_size) || (blkcnt % mmc->erase_grp_size))
		mmcdbg("\n\nCaution! Your devices Erase group is 0x%x\n"
			"The erase range would be change to 0x%x~0x%x\n\n",
		       mmc->erase_grp_size, start & ~(mmc->erase_grp_size - 1),
		       ((start + blkcnt + mmc->erase_grp_size)
		       & ~(mmc->erase_grp_size - 1)) - 1);

	while (blk < blkcnt) {
		blk_r = ((blkcnt - blk) > mmc->erase_grp_size) ?
			mmc->erase_grp_size : (blkcnt - blk);
		err = mmc_erase_t(mmc, start + blk, blk_r);
		if (err)
			break;

		blk += blk_r;
	}
	return blk;
	*/
}

static unsigned long
mmc_write_blocks(struct mmc *mmc, unsigned long start, unsigned blkcnt, const void*src)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int timeout = 1000;

	if ((start + blkcnt) > mmc->lba) {
		mmcinfo("mmc %d: block number 0x%lx exceeds max(0x%lx)\n",mmc->control_num,
			start + blkcnt, mmc->lba);
		return 0;
	}

	if (blkcnt > 1)
		cmd.cmdidx = MMC_CMD_WRITE_MULTIPLE_BLOCK;
	else
		cmd.cmdidx = MMC_CMD_WRITE_SINGLE_BLOCK;

	if (mmc->high_capacity)
		cmd.cmdarg = start;
	else
		cmd.cmdarg = start * mmc->write_bl_len;

	cmd.resp_type = MMC_RSP_R1;
	cmd.flags = 0;

	data.b.src = src;
	data.blocks = blkcnt;
	data.blocksize = mmc->write_bl_len;
	data.flags = MMC_DATA_WRITE;

	if (mmc_send_cmd(mmc, &cmd, &data)) {
		mmcinfo("mmc %d mmc write failed\n",mmc->control_num);
		return 0;
	}

	/* SPI multiblock writes terminate using a special
	 * token, not a STOP_TRANSMISSION request.
	 */
	if (!mmc_host_is_spi(mmc) && blkcnt > 1) {
		cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
		cmd.cmdarg = 0;
		cmd.resp_type = MMC_RSP_R1b;
		cmd.flags = 0;
		if (mmc_send_cmd(mmc, &cmd, NULL)) {
			mmcinfo("mmc %d fail to send stop cmd\n",mmc->control_num);
			return 0;
		}


	}

	/* Waiting for the ready status */
	mmc_send_status(mmc, timeout);
	return blkcnt;
}

unsigned long
mmc_bwrite(int dev_num, unsigned long start, unsigned blkcnt, const void*src)
{
	unsigned cur, blocks_todo = blkcnt;

	struct mmc *mmc = find_mmc_device(dev_num);

	if (blkcnt == 0){
		mmcinfo("mmc %d blkcnt should not be 0\n",dev_num);
		return 0;
	}
	if (!mmc){
		mmcinfo("Can not found device %d\n",dev_num);
		return 0;
	}

	if (mmc_set_blocklen(mmc, mmc->write_bl_len)){
		mmcinfo("mmc %d set block len failed\n",mmc->control_num);
//		sunxi_mmc_exit(dev_num);
//		if(sunxi_mmc_init(dev_num,4)<0){
//			mmcinfo("re init failed\n");
//			return 0;
//		}
		return 0;
	}

	do {
		cur = (blocks_todo > mmc->b_max) ?  mmc->b_max : blocks_todo;
		if(mmc_write_blocks(mmc, start, cur, src) != cur){
			mmcinfo("mmc %d write block failed\n",mmc->control_num);
			return 0;
		}
		blocks_todo -= cur;
		start += cur;
//		src += cur * mmc->write_bl_len;
		src = (char*)src + cur * mmc->write_bl_len;
	} while (blocks_todo > 0);

	return blkcnt;
}
#endif

int mmc_read_blocks(struct mmc *mmc, void *dst, unsigned long start, unsigned blkcnt)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int timeout = 1000;

	if (blkcnt > 1)
		cmd.cmdidx = MMC_CMD_READ_MULTIPLE_BLOCK;
	else
		cmd.cmdidx = MMC_CMD_READ_SINGLE_BLOCK;

	if (mmc->high_capacity)
		cmd.cmdarg = start;
	else
		cmd.cmdarg = start * mmc->read_bl_len;

	cmd.resp_type = MMC_RSP_R1;
	cmd.flags = 0;

	data.b.dest = dst;
	data.blocks = blkcnt;
	data.blocksize = mmc->read_bl_len;
	data.flags = MMC_DATA_READ;

	if (mmc_send_cmd(mmc, &cmd, &data)){
		mmcinfo("mmc %d  read blcok failed\n",mmc->control_num);
		return 0;
	}

	if (blkcnt > 1) {
		cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
		cmd.cmdarg = 0;
		cmd.resp_type = MMC_RSP_R1b;
		cmd.flags = 0;
		if (mmc_send_cmd(mmc, &cmd, NULL)) {
			mmcinfo("mmc %d fail to send stop cmd\n",mmc->control_num);
			return 0;
		}

		/* Waiting for the ready status */
		mmc_send_status(mmc, timeout);
	}

	return blkcnt;
}

unsigned long
mmc_bread(int dev_num, unsigned long start, unsigned blkcnt, void *dst)
{
	unsigned cur, blocks_todo = blkcnt;
	struct mmc *mmc = find_mmc_device(dev_num);

	if (blkcnt == 0){
		mmcinfo("mmc %d blkcnt should not be 0\n",mmc->control_num);
		return 0;
	}
	if (!mmc){
		mmcinfo("Can not find mmc dev %d\n",dev_num);
		return 0;
	}

	if ((start + blkcnt) > mmc->lba) {
		mmcinfo("mmc %d: block number 0x%x exceeds max(0x%x)\n",mmc->control_num,
			(unsigned int)(start + blkcnt), (unsigned int)mmc->lba);
		return 0;
	}

	if (mmc_set_blocklen(mmc, mmc->read_bl_len)){
		mmcinfo("mmc %d Set block len failed\n",mmc->control_num);
		return 0;
	}

	do {
		cur = (blocks_todo > mmc->b_max) ?  mmc->b_max : blocks_todo;
		if(mmc_read_blocks(mmc, dst, start, cur) != cur){
			mmcinfo("mmc %d block read failed\n",mmc->control_num);
			return 0;
		}
		blocks_todo -= cur;
		start += cur;
//		dst += cur * mmc->read_bl_len;
		dst = (char*)dst + cur * mmc->read_bl_len;
	} while (blocks_todo > 0);

	return blkcnt;
}

int mmc_go_idle(struct mmc* mmc)
{
	struct mmc_cmd cmd;
	int err;

	__msdelay(1);

	cmd.cmdidx = MMC_CMD_GO_IDLE_STATE;
	cmd.cmdarg = 0;
	cmd.resp_type = MMC_RSP_NONE;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err){
		mmcinfo("mmc %d go idle failed\n",mmc->control_num);
		return err;
	}

	__msdelay(2);

	return 0;
}

int
sd_send_op_cond(struct mmc *mmc)
{
	int timeout = 1000;
	int err;
	struct mmc_cmd cmd;

	do {
		cmd.cmdidx = MMC_CMD_APP_CMD;
		cmd.resp_type = MMC_RSP_R1;
		cmd.cmdarg = 0;
		cmd.flags = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("mmc %d send app cmd failed\n",mmc->control_num);
			return err;
		}

		cmd.cmdidx = SD_CMD_APP_SEND_OP_COND;
		cmd.resp_type = MMC_RSP_R3;

		/*
		 * Most cards do not answer if some reserved bits
		 * in the ocr are set. However, Some controller
		 * can set bit 7 (reserved for low voltages), but
		 * how to manage low voltages SD card is not yet
		 * specified.
		 */
		cmd.cmdarg = mmc_host_is_spi(mmc) ? 0 :
			(mmc->voltages & 0xff8000);

		if (mmc->version == SD_VERSION_2)
			cmd.cmdarg |= OCR_HCS;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("mmc %d send cmd41 failed\n",mmc->control_num);
			return err;
		}

		__msdelay(1);
	} while ((!(cmd.response[0] & OCR_BUSY)) && timeout--);

	if (timeout <= 0){
		mmcinfo("mmc %d wait card init failed\n",mmc->control_num);
		return UNUSABLE_ERR;
	}

	if (mmc->version != SD_VERSION_2)
		mmc->version = SD_VERSION_1_0;

	if (mmc_host_is_spi(mmc)) { /* read OCR for spi */
		cmd.cmdidx = MMC_CMD_SPI_READ_OCR;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = 0;
		cmd.flags = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("mmc %d spi read ocr failed\n",mmc->control_num);
			return err;
		}
	}

	mmc->ocr = cmd.response[0];

	mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
	mmc->rca = 0;

	return 0;
}

int mmc_send_op_cond(struct mmc *mmc)
{
	int timeout = 10000;
	struct mmc_cmd cmd;
	int err;

	/* Some cards seem to need this */
	mmc_go_idle(mmc);

	/* Asking to the card its capabilities */
	cmd.cmdidx = MMC_CMD_SEND_OP_COND;
	cmd.resp_type = MMC_RSP_R3;
	cmd.cmdarg = 0x40ff8000;//foresee
	cmd.flags = 0;

  //mmcinfo("mmc send op cond arg not zero !!!\n");
	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err){
		mmcinfo("mmc %d send op cond failed\n",mmc->control_num);
		return err;
	}

	__msdelay(1);

	do {
		cmd.cmdidx = MMC_CMD_SEND_OP_COND;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = (mmc_host_is_spi(mmc) ? 0 :
				(mmc->voltages &
				(cmd.response[0] & OCR_VOLTAGE_MASK)) |
				(cmd.response[0] & OCR_ACCESS_MODE));

		if (mmc->host_caps & MMC_MODE_HC)
			cmd.cmdarg |= OCR_HCS;

		cmd.flags = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("mmc %d send op cond failed\n",mmc->control_num);
			return err;
		}

		__msdelay(1);
	} while (!(cmd.response[0] & OCR_BUSY) && timeout--);

	if (timeout <= 0){
		mmcinfo("mmc %d wait for mmc init failed\n",mmc->control_num);
		return UNUSABLE_ERR;
	}

	if (mmc_host_is_spi(mmc)) { /* read OCR for spi */
		cmd.cmdidx = MMC_CMD_SPI_READ_OCR;
		cmd.resp_type = MMC_RSP_R3;
		cmd.cmdarg = 0;
		cmd.flags = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);
		if (err)
			return err;
	}

	mmc->version = MMC_VERSION_UNKNOWN;
	mmc->ocr = cmd.response[0];

	mmc->high_capacity = ((mmc->ocr & OCR_HCS) == OCR_HCS);
	mmc->rca = 1;

	return 0;
}


int mmc_send_ext_csd(struct mmc *mmc, char *ext_csd)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int err;

	/* Get the Card Status Register */
	cmd.cmdidx = MMC_CMD_SEND_EXT_CSD;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0;
	cmd.flags = 0;

	data.b.dest = ext_csd;
	data.blocks = 1;
	data.blocksize = 512;
	data.flags = MMC_DATA_READ;

	err = mmc_send_cmd(mmc, &cmd, &data);
	if(err)
		mmcinfo("mmc %d send ext csd failed\n",mmc->control_num);

	return err;
}

int mmc_update_phase(struct mmc *mmc)
{
	if(mmc->update_phase == NULL)
		return 0;

	return mmc->update_phase(mmc);
}

int mmc_switch(struct mmc *mmc, u8 set, u8 index, u8 value)
{
	struct mmc_cmd cmd;
	int timeout = 1000;
	int ret;

	cmd.cmdidx = MMC_CMD_SWITCH;
	cmd.resp_type = MMC_RSP_R1b;
	cmd.cmdarg = (MMC_SWITCH_MODE_WRITE_BYTE << 24) |
				 (index << 16) |
				 (value << 8);
	cmd.flags = 0;

	ret = mmc_send_cmd(mmc, &cmd, NULL);
	if(ret){
		mmcinfo("mmc %d switch failed\n",mmc->control_num);
	}

	/* for re-update sample phase */
	ret = mmc_update_phase(mmc);
	if (ret) {
		mmcinfo("mmc_switch: update clock failed after send cmd6\n");
		return ret;
	}

	/* Waiting for the ready status */
	mmc_send_status(mmc, timeout);

	return ret;

}

int mmc_change_freq(struct mmc *mmc)
{
	char ext_csd[512];
	char cardtype;
	int err;
	int retry = 5;

	mmc->card_caps = 0;

	if (mmc_host_is_spi(mmc))
		return 0;

	/* Only version 4 supports high-speed */
	if (mmc->version < MMC_VERSION_4)
		return 0;

	mmc->card_caps |= MMC_MODE_4BIT|MMC_MODE_8BIT;
	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err){
		mmcinfo("mmc %d get ext csd failed\n",mmc->control_num);
		return err;
	}

	cardtype = ext_csd[196] & 0xff;

	//retry for Toshiba emmc,for the first time Toshiba emmc change to HS
	//it will return response crc err,so retry
	do{
		err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_HS_TIMING, 1);
		if(!err){
			break;
		}
		mmcinfo("retry mmc switch(cmd6)\n");
	}while(retry--);

	if (err){
		mmcinfo("mmc %d change to hs failed\n",mmc->control_num);
		return err;
	}

	/* Now check to see that it worked */
	err = mmc_send_ext_csd(mmc, ext_csd);

	if (err){
		mmcinfo("mmc %d send ext csd faild\n",mmc->control_num);
		return err;
	}

	/* No high-speed support */
	if (!ext_csd[185])
		return 0;

	/* High Speed is set, there are two types: 52MHz and 26MHz */
	if (cardtype & EXT_CSD_CARD_TYPE_HS) { //EXT_CSD_CARD_TYPE_52
		if (cardtype & EXT_CSD_CARD_TYPE_DDR_52) {
			mmcdbg("%s: get ddr OK!\n", __FUNCTION__);
			mmc->card_caps |= MMC_MODE_DDR_52MHz;
			mmc->speed_mode = HSDDR52_DDR50;
		} else
			mmc->speed_mode = HSSDR52_SDR25;
		mmc->card_caps |= MMC_MODE_HS_52MHz | MMC_MODE_HS;
	} else {
		mmc->card_caps |= MMC_MODE_HS;
		mmc->speed_mode = DS26_SDR12;
	}

	return 0;
}

int mmc_switch_part(int dev_num, unsigned int part_num)
{
	struct mmc *mmc = find_mmc_device(dev_num);

	if (!mmc)
		return -1;

	return mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL, EXT_CSD_PART_CONF,
			  (mmc->part_config & ~PART_ACCESS_MASK)
			  | (part_num & PART_ACCESS_MASK));
}

int sd_switch(struct mmc *mmc, int mode, int group, u8 value, u8 *resp)
{
	struct mmc_cmd cmd;
	struct mmc_data data;

	/* Switch the frequency */
	cmd.cmdidx = SD_CMD_SWITCH_FUNC;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = (mode << 31) | 0xffffff;
	cmd.cmdarg &= ~(0xf << (group * 4));
	cmd.cmdarg |= value << (group * 4);
	cmd.flags = 0;

	data.b.dest = (char *)resp;
	data.blocksize = 64;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	return mmc_send_cmd(mmc, &cmd, &data);
}


int sd_change_freq(struct mmc *mmc)
{
	int err;
	struct mmc_cmd cmd;
	u32 scr[2];
	u32 switch_status[16];
	struct mmc_data data;
	int timeout;

	mmc->card_caps = 0;

	if (mmc_host_is_spi(mmc))
		return 0;

	/* Read the SCR to find out if this card supports higher speeds */
	cmd.cmdidx = MMC_CMD_APP_CMD;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err){
		mmcinfo("mmc %d Send app cmd failed\n",mmc->control_num);
		return err;
	}

	cmd.cmdidx = SD_CMD_APP_SEND_SCR;
	cmd.resp_type = MMC_RSP_R1;
	cmd.cmdarg = 0;
	cmd.flags = 0;

	timeout = 3;

retry_scr:
	data.b.dest = (char *)&scr;
	data.blocksize = 8;
	data.blocks = 1;
	data.flags = MMC_DATA_READ;

	err = mmc_send_cmd(mmc, &cmd, &data);

	if (err) {
		if (timeout--)
			goto retry_scr;

		mmcinfo("mmc %d Send scr failed\n",mmc->control_num);
		return err;
	}

	mmc->scr[0] = __mmc_be32_to_cpu(scr[0]);
	mmc->scr[1] = __mmc_be32_to_cpu(scr[1]);

	switch ((mmc->scr[0] >> 24) & 0xf) {
		case 0:
			mmc->version = SD_VERSION_1_0;
			break;
		case 1:
			mmc->version = SD_VERSION_1_10;
			break;
		case 2:
			mmc->version = SD_VERSION_2;
			break;
		default:
			mmc->version = SD_VERSION_1_0;
			break;
	}

	if (mmc->scr[0] & SD_DATA_4BIT)
		mmc->card_caps |= MMC_MODE_4BIT;

	/* Version 1.0 doesn't support switching */
	if (mmc->version == SD_VERSION_1_0)
		return 0;

	timeout = 4;
	while (timeout--) {
		err = sd_switch(mmc, SD_SWITCH_CHECK, 0, 1,
				(u8 *)&switch_status);

		if (err){
			mmcinfo("mmc %d Check high speed status faild\n",mmc->control_num);
			return err;
		}

		/* The high-speed function is busy.  Try again */
		if (!(__be32_to_cpu(switch_status[7]) & SD_HIGHSPEED_BUSY))
			break;
	}

	/* If high-speed isn't supported, we return */
	if (!(__be32_to_cpu(switch_status[3]) & SD_HIGHSPEED_SUPPORTED))
		return 0;

	err = sd_switch(mmc, SD_SWITCH_SWITCH, 0, 1, (u8 *)&switch_status);

	if (err){
		mmcinfo("mmc %d switch to high speed failed\n",mmc->control_num);
		return err;
	}

	err = mmc_update_phase(mmc);
	if (err) {
		mmcinfo("update clock failed after send cmd6 to switch to sd high speed mode\n");
		return err;
	}

	if ((__be32_to_cpu(switch_status[4]) & 0x0f000000) == 0x01000000) {
		mmc->card_caps |= MMC_MODE_HS;
		mmc->speed_mode = HSSDR52_SDR25;
	}

	return 0;
}

/* frequency bases */
/* divided by 10 to be nice to platforms without floating point */
static const int fbase[] = {
	10000,
	100000,
	1000000,
	10000000,
};

/* Multiplier values for TRAN_SPEED.  Multiplied by 10 to be nice
 * to platforms without floating point.
 */
static const int multipliers[] = {
	0,	/* reserved */
	10,
	12,
	13,
	15,
	20,
	25,
	30,
	35,
	40,
	45,
	50,
	55,
	60,
	70,
	80,
};

void mmc_set_ios(struct mmc *mmc)
{
	mmc->set_ios(mmc);
}

void mmc_set_clock(struct mmc *mmc, u32 clock)
{
	if (clock > mmc->f_max)
		clock = mmc->f_max;

	if (clock < mmc->f_min)
		clock = mmc->f_min;

	mmc->clock = clock;

	mmc_set_ios(mmc);
}

#if 0
void mmc_set_bus_mode(struct mmc *mmc, u32 ddr)
{
	mmc->io_mode= ddr;
	mmc_set_ios(mmc);
}
#endif

void mmc_set_bus_width(struct mmc *mmc, u32 width)
{
	mmc->bus_width = width;

	mmc_set_ios(mmc);
}

int mmc_startup(struct mmc *mmc)
{
	int err;
	u32 mult, freq;
	u64 cmult, csize, capacity;
	struct mmc_cmd cmd;
	char ext_csd[512];
	int timeout = 1000;
	char *spd_name[] = {"DS26/SDR12", "HSSDR52/SDR25", "HSDDR52/DDR50", "HS200/SDR104", "HS400"};

	/* Put the Card in Identify Mode */
	cmd.cmdidx = mmc_host_is_spi(mmc) ? MMC_CMD_SEND_CID :
		MMC_CMD_ALL_SEND_CID; /* cmd not supported in spi */
	cmd.resp_type = MMC_RSP_R2;
	cmd.cmdarg = 0;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err){
		mmcinfo("mmc %d Put the Card in Identify Mode failed\n",mmc->control_num);
		return err;
	}

	memcpy(mmc->cid, cmd.response, 16);

	/*
	 * For MMC cards, set the Relative Address.
	 * For SD cards, get the Relatvie Address.
	 * This also puts the cards into Standby State
	 */
	if (!mmc_host_is_spi(mmc)) { /* cmd not supported in spi */
		cmd.cmdidx = SD_CMD_SEND_RELATIVE_ADDR;
		cmd.cmdarg = mmc->rca << 16;
		cmd.resp_type = MMC_RSP_R6;
		cmd.flags = 0;

		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("mmc %d send rca failed\n",mmc->control_num);
			return err;
		}

		if (IS_SD(mmc))
			mmc->rca = (cmd.response[0] >> 16) & 0xffff;
	}

	/* Get the Card-Specific Data */
	cmd.cmdidx = MMC_CMD_SEND_CSD;
	cmd.resp_type = MMC_RSP_R2;
	cmd.cmdarg = mmc->rca << 16;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	/* Waiting for the ready status */
	mmc_send_status(mmc, timeout);

	if (err){
		mmcinfo("mmc %d get csd failed\n",mmc->control_num);
		return err;
	}

	mmc->csd[0] = cmd.response[0];
	mmc->csd[1] = cmd.response[1];
	mmc->csd[2] = cmd.response[2];
	mmc->csd[3] = cmd.response[3];

	if (mmc->version == MMC_VERSION_UNKNOWN) {
		int version = (cmd.response[0] >> 26) & 0xf;

		switch (version) {
			case 0:
				mmc->version = MMC_VERSION_1_2;
				break;
			case 1:
				mmc->version = MMC_VERSION_1_4;
				break;
			case 2:
				mmc->version = MMC_VERSION_2_2;
				break;
			case 3:
				mmc->version = MMC_VERSION_3;
				break;
			case 4:
				mmc->version = MMC_VERSION_4;
				break;
			default:
				mmc->version = MMC_VERSION_1_2;
				break;
		}
	}

	/* divide frequency by 10, since the mults are 10x bigger */
	freq = fbase[(cmd.response[0] & 0x7)];
	mult = multipliers[((cmd.response[0] >> 3) & 0xf)];

	mmc->tran_speed = freq * mult;

	mmc->read_bl_len = 1 << ((cmd.response[1] >> 16) & 0xf);

	if (IS_SD(mmc))
		mmc->write_bl_len = mmc->read_bl_len;
	else
		mmc->write_bl_len = 1 << ((cmd.response[3] >> 22) & 0xf);

	if (mmc->high_capacity) {
		csize = (mmc->csd[1] & 0x3f) << 16
			| (mmc->csd[2] & 0xffff0000) >> 16;
		cmult = 8;
	} else {
		csize = (mmc->csd[1] & 0x3ff) << 2
			| (mmc->csd[2] & 0xc0000000) >> 30;
		cmult = (mmc->csd[2] & 0x00038000) >> 15;
	}

	mmc->capacity = (csize + 1) << (cmult + 2);
	mmc->capacity *= mmc->read_bl_len;

	if (mmc->read_bl_len > 512)
		mmc->read_bl_len = 512;

	if (mmc->write_bl_len > 512)
		mmc->write_bl_len = 512;

	mmc_set_clock(mmc, 25000000);

	/* Select the card, and put it into Transfer Mode */
	if (!mmc_host_is_spi(mmc)) { /* cmd not supported in spi */
		cmd.cmdidx = MMC_CMD_SELECT_CARD;
		cmd.resp_type = MMC_RSP_R1b;
		cmd.cmdarg = mmc->rca << 16;
		cmd.flags = 0;
		err = mmc_send_cmd(mmc, &cmd, NULL);

		if (err){
			mmcinfo("Select the card failed\n");
			return err;
		}
	}

	/*
	 * For SD, its erase group is always one sector
	 */
	mmc->erase_grp_size = 1;
	mmc->part_config = MMCPART_NOAVAILABLE;
	if (!IS_SD(mmc) && (mmc->version >= MMC_VERSION_4)) {
		/* check  ext_csd version and capacity */
		err = mmc_send_ext_csd(mmc, ext_csd);
		if(!err){
				/* update mmc version */
			switch (ext_csd[192]) {
				case 0:
					mmc->version = MMC_VERSION_4;
					break;
				case 1:
					mmc->version = MMC_VERSION_4_1;
					break;
				case 2:
					mmc->version = MMC_VERSION_4_2;
					break;
				case 3:
					mmc->version = MMC_VERSION_4_3;
					break;
				case 5:
					mmc->version = MMC_VERSION_4_41;
					break;
				case 6:
					mmc->version = MMC_VERSION_4_5;
					break;
				case 7:
					mmc->version = MMC_VERSION_5_0;
					break;
				case 8:
					mmc->version = MMC_VERSION_5_1;
					break;
			}
		}

		if (!err & (ext_csd[192] >= 2)) {
			/*
			 * According to the JEDEC Standard, the value of
			 * ext_csd's capacity is valid if the value is more
			 * than 2GB
			 */
			capacity = ext_csd[212] << 0 | ext_csd[213] << 8 |
				   ext_csd[214] << 16 | ext_csd[215] << 24;
			capacity *= 512;
			if ((capacity >> 20) > 2 * 1024)
				mmc->capacity = capacity;
		}

		/*
		 * Check whether GROUP_DEF is set, if yes, read out
		 * group size from ext_csd directly, or calculate
		 * the group size from the csd value.
		 */
		if (ext_csd[175])
			mmc->erase_grp_size = ext_csd[224] * 512 * 1024;
		else {
			int erase_gsz, erase_gmul;
			erase_gsz = (mmc->csd[2] & 0x00007c00) >> 10;
			erase_gmul = (mmc->csd[2] & 0x000003e0) >> 5;
			mmc->erase_grp_size = (erase_gsz + 1)
				* (erase_gmul + 1);
		}

		/* store the partition info of emmc */
		if (ext_csd[160] & PART_SUPPORT)
			mmc->part_config = ext_csd[179];
	}

	if (IS_SD(mmc))
		err = sd_change_freq(mmc);
	else
		err = mmc_change_freq(mmc);

	if (err){
		mmcinfo("mmc %d Change speed mode failed\n",mmc->control_num);
		return err;
	}

	/* for re-update sample phase */
	err = mmc_update_phase(mmc);
	if (err)
	{
		mmcinfo("update clock failed\n");
		return err;
	}

	mmcdbg("mmc->card_caps 0x%x, ddr caps:0x%x\n", mmc->card_caps, mmc->card_caps & MMC_MODE_DDR_52MHz);
	/* Restrict card's capabilities by what the host can do */
	mmc->card_caps &= mmc->host_caps;
	mmcdbg("mmc->card_caps 0x%x, ddr caps:0x%x\n", mmc->card_caps, mmc->card_caps & MMC_MODE_DDR_52MHz);
	if (!(mmc->card_caps & MMC_MODE_DDR_52MHz) && !IS_SD(mmc)) {
		if (mmc->speed_mode == HSDDR52_DDR50)
			mmc->speed_mode = HSSDR52_SDR25;
		else
			mmc->speed_mode = DS26_SDR12;
	}

	if (IS_SD(mmc)) {
		if (mmc->card_caps & MMC_MODE_4BIT) {
			cmd.cmdidx = MMC_CMD_APP_CMD;
			cmd.resp_type = MMC_RSP_R1;
			cmd.cmdarg = mmc->rca << 16;
			cmd.flags = 0;

			err = mmc_send_cmd(mmc, &cmd, NULL);
			if (err){
				mmcinfo("mmc %d send app cmd failed\n",mmc->control_num);
				return err;
			}

			cmd.cmdidx = SD_CMD_APP_SET_BUS_WIDTH;
			cmd.resp_type = MMC_RSP_R1;
			cmd.cmdarg = 2;
			cmd.flags = 0;
			err = mmc_send_cmd(mmc, &cmd, NULL);
			if (err){
				mmcinfo("mmc %d sd set bus width failed\n",mmc->control_num);
				return err;
			}

			mmc_set_bus_width(mmc, 4);
		}

		if (mmc->card_caps & MMC_MODE_HS)
			mmc->tran_speed = 50000000;
		else
			mmc->tran_speed = 25000000;
	} else {

		if (mmc->card_caps & MMC_MODE_8BIT) {

			/* Set the card to use 8 bit*/
			if( (mmc->card_caps & MMC_MODE_DDR_52MHz) ){
				mmcdbg("8bit ddr!!! \n");
				/* Set the card to use 8 bit ddr*/
				err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
						EXT_CSD_BUS_WIDTH,
						EXT_CSD_BUS_DDR_8);
				if (err){
					mmcinfo("mmc %d switch bus width failed\n", mmc->control_num);
					return err;
				}
				//mmc_set_bus_mode(mmc,1);
				mmc_set_bus_width(mmc, 8);
			}else{
				/* Set the card to use 8 bit*/
				err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
						EXT_CSD_BUS_WIDTH,
						EXT_CSD_BUS_WIDTH_8);

				if (err){
					mmcinfo("mmc %d switch bus width8 failed\n", mmc->control_num);
					return err;
				}
				mmc_set_bus_width(mmc, 8);
			}
		}else if (mmc->card_caps & MMC_MODE_4BIT) {
			if ( (mmc->card_caps & MMC_MODE_DDR_52MHz) ){
				mmcdbg("4bit bus ddr!!! \n");
				/* Set the card to use 4 bit ddr*/
				err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
						EXT_CSD_BUS_WIDTH,
						EXT_CSD_BUS_DDR_4);
				if (err){
					mmcinfo("mmc %d switch bus width failed\n", mmc->control_num);
					return err;
				}
				//mmc_set_bus_mode(mmc,1);
				mmc_set_bus_width(mmc, 4);
			}else{
				/* Set the card to use 4 bit*/
				err = mmc_switch(mmc, EXT_CSD_CMD_SET_NORMAL,
						EXT_CSD_BUS_WIDTH,
						EXT_CSD_BUS_WIDTH_4);
				if (err){
					mmcinfo("mmc %d switch bus width failed\n", mmc->control_num);
					return err;
				}
				mmc_set_bus_width(mmc, 4);
			}
		}

		if (mmc->card_caps & MMC_MODE_DDR_52MHz) {
			mmc->tran_speed = 52000000;
		} else if (mmc->card_caps & MMC_MODE_HS) {
			if (mmc->card_caps & MMC_MODE_HS_52MHz)
				mmc->tran_speed = 52000000;
			else
				mmc->tran_speed = 26000000;
		} else {
			mmc->tran_speed = 26000000;
		}
	}
	mmcdbg("%s: set clock %d\n", __FUNCTION__, mmc->tran_speed);
	mmc_set_clock(mmc, mmc->tran_speed);

	/* fill in device description */
	mmc->blksz = mmc->read_bl_len;
	//mmc->lba = mmc->capacity/mmc->read_bl_len;   //for compiler error
	mmc->lba = mmc->capacity>>9;

	if(!IS_SD(mmc)){
		switch(mmc->version)
		{
			case MMC_VERSION_1_2:
				mmcinfo("MMC 1.2\n");
				break;
			case MMC_VERSION_1_4:
				mmcinfo("MMC 1.4\n");
				break;
			case MMC_VERSION_2_2:
				mmcinfo("MMC 2.2\n");
				break;
			case MMC_VERSION_3:
				mmcinfo("MMC 3.0\n");
				break;
			case MMC_VERSION_4:
				mmcinfo("MMC 4.0\n");
				break;
			case MMC_VERSION_4_1:
				mmcinfo("MMC 4.1\n");
				break;
			case MMC_VERSION_4_2:
				mmcinfo("MMC 4.2\n");
				break;
			case MMC_VERSION_4_3:
				mmcinfo("MMC 4.3\n");
				break;
			case MMC_VERSION_4_41:
				mmcinfo("MMC 4.41\n");
				break;
			case MMC_VERSION_4_5:
				mmcinfo("MMC 4.5\n");
				break;
			case MMC_VERSION_5_0:
				mmcinfo("MMC 5.0\n");
				break;
			case MMC_VERSION_5_1:
				mmcinfo("MMC 5.1\n");
				break;
			default:
				mmcinfo("Unknow MMC ver\n");
				break;
		}
	}

#if 0
	if (IS_SD(mmc)){
		sprintf(mmc->revision, "PRV %d.%d", mmc->cid[2] >> 28,
			(mmc->cid[2] >> 24) & 0xf);
	} else {
		sprintf(mmc->revision, "PRV %d.%d", (mmc->cid[2] >> 20) & 0xf,
			(mmc->cid[2] >> 16) & 0xf);
	}
	mmcinfo("%s\n", mmc->revision);

	if (IS_SD(mmc)) {
		mmcinfo("MDT m-%d y-%d\n", ((mmc->cid[3] >> 8) & 0xF), (((mmc->cid[3] >> 12) & 0xFF) + 2000));
	} else {
		if (ext_csd[192] > 4) {
			mmcinfo("MDT m-%d y-%d\n", ((mmc->cid[3] >> 12) & 0xF),
				(((mmc->cid[3] >> 8) & 0xF) < 13) ? (((mmc->cid[3] >> 8) & 0xF) + 2013) : (((mmc->cid[3] >> 8) & 0xF) + 1997));
		} else {
			mmcinfo("MDT m-%d y-%d\n", ((mmc->cid[3] >> 12) & 0xF), (((mmc->cid[3] >> 8) & 0xF) + 1997));
		}
	}
#endif

	mmcinfo("%s %d bit\n", spd_name[mmc->speed_mode], mmc->bus_width);
	mmcinfo("%d Hz\n", mmc->clock);
	mmcinfo("%d MB\n", mmc->lba >> 11);
	return 0;
}

int mmc_send_if_cond(struct mmc *mmc)
{
	struct mmc_cmd cmd;
	int err;

	cmd.cmdidx = SD_CMD_SEND_IF_COND;
	/* We set the bit if the host supports voltages between 2.7 and 3.6 V */
	cmd.cmdarg = ((mmc->voltages & 0xff8000) != 0) << 8 | 0xaa;
	cmd.resp_type = MMC_RSP_R7;
	cmd.flags = 0;

	err = mmc_send_cmd(mmc, &cmd, NULL);

	if (err){
		mmcinfo("mmc %d send if cond failed\n",mmc->control_num);
		return err;
	}

	if ((cmd.response[0] & 0xff) != 0xaa)
		return UNUSABLE_ERR;
	else
		mmc->version = SD_VERSION_2;

	return 0;
}

static lbaint_t mmc_write_blocks(struct mmc *mmc, lbaint_t start,
		lbaint_t blkcnt, const void *src)
{
	struct mmc_cmd cmd;
	struct mmc_data data;
	int timeout = 1000;

	//if ((start + blkcnt) > mmc_get_blk_desc(mmc)->lba) {
		//printf("MMC: block number 0x" LBAF " exceeds max(0x" LBAF ")\n",
		  //     start + blkcnt, mmc_get_blk_desc(mmc)->lba);
	//	return 0;
//	}

	if (blkcnt == 0)
		return 0;
	else if (blkcnt == 1)
		cmd.cmdidx = MMC_CMD_WRITE_SINGLE_BLOCK;
	else
		cmd.cmdidx = MMC_CMD_WRITE_MULTIPLE_BLOCK;

	if (mmc->high_capacity)
		cmd.cmdarg = start;
	else
		cmd.cmdarg = start * mmc->write_bl_len;

	cmd.resp_type = MMC_RSP_R1;

	data.b.src = src;
	data.blocks = blkcnt;
	data.blocksize = mmc->write_bl_len;
	data.flags = MMC_DATA_WRITE;


	if (mmc_send_cmd(mmc, &cmd, &data)) {
		printf("mmc write failed\n");
		return 0;
	}

	/* SPI multiblock writes terminate using a special
	 * token, not a STOP_TRANSMISSION request.
	 */
	/*
	if (!mmc_host_is_spi(mmc) && blkcnt > 1) {
		cmd.cmdidx = MMC_CMD_STOP_TRANSMISSION;
		cmd.cmdarg = 0;
		cmd.resp_type = MMC_RSP_R1b;
		if (mmc_send_cmd(mmc, &cmd, NULL)) {
			printf("mmc fail to send stop cmd\n");
			return 0;
		}
	}
	*/
	/* Waiting for the ready status */
	if (mmc_send_status(mmc, timeout))
		return 0;

	return blkcnt;
}

int mmc_init(struct mmc *mmc)
{
	int err;
	struct boot_sdmmc_private_info_t *priv_info =
		(struct boot_sdmmc_private_info_t *)(mmc_config_addr + SDMMC_PRIV_INFO_ADDR_OFFSET);

	if (mmc->has_init){
		mmcinfo("mmc %d Has init\n",mmc->control_num);
		return 0;
	}

	err = mmc->init(mmc);
	if (err){
		mmcinfo("mmc %d host init failed\n",mmc->control_num);
		return err;
	}

	mmc_set_bus_width(mmc, 1);
	mmc_set_clock(mmc, 1);

	/* Reset the Card */
	err = mmc_go_idle(mmc);
	if (err){
		mmcinfo("mmc %d reset card failed\n",mmc->control_num);
		return err;
	}

	/* The internal partition reset to user partition(0) at every CMD0*/
	mmc->part_num = 0;

#if 0
	mmcinfo("***Try SD card %d***\n",mmc->control_num);
	/* Test for SD version 2 */
	err = mmc_send_if_cond(mmc);

	/* Now try to get the SD card's operating condition */
	err = sd_send_op_cond(mmc);

	/* If the command timed out, we check for an MMC card */
	if(err){
		mmcinfo("***Try MMC card %d***\n",mmc->control_num);
		err = mmc_send_op_cond(mmc);

		if (err) {
			mmcinfo("mmc %d Card did not respond to voltage select!\n",mmc->control_num);
			mmcinfo("***SD/MMC %d init error!!!***\n",mmc->control_num);
			return UNUSABLE_ERR;
		}
	}
#else
	if (priv_info->card_type == CARD_TYPE_SD)
	{
		mmcinfo("***Try SD card %d***\n",mmc->control_num);
		/* Test for SD version 2 */
		err = mmc_send_if_cond(mmc);

		/* Now try to get the SD card's operating condition */
		err = sd_send_op_cond(mmc);

		if (err) {
			mmcinfo("SD card %d Card did not respond to voltage select!\n",mmc->control_num);
			mmcinfo("***SD/MMC %d init error!!!***\n",mmc->control_num);
			return UNUSABLE_ERR;
		}
	}
	else if (priv_info->card_type == CARD_TYPE_MMC)
	{
		/* If the command timed out, we check for an MMC card */
		mmcinfo("***Try MMC card %d***\n",mmc->control_num);
		err = mmc_send_op_cond(mmc);

		if (err) {
			mmcinfo("MMC card %d Card did not respond to voltage select!\n",mmc->control_num);
			mmcinfo("***SD/MMC %d init error!!!***\n",mmc->control_num);
			return UNUSABLE_ERR;
		}
	}
	else
	{
		mmcinfo("Wrong media type 0x%x\n", priv_info->card_type);

		mmcinfo("***Try SD card %d***\n",mmc->control_num);
		/* Test for SD version 2 */
		err = mmc_send_if_cond(mmc);

		/* Now try to get the SD card's operating condition */
		err = sd_send_op_cond(mmc);

		/* If the command timed out, we check for an MMC card */
		if(err){
			mmcinfo("***Try MMC card %d***\n",mmc->control_num);
			err = mmc_send_op_cond(mmc);

			if (err) {
				mmcinfo("mmc %d Card did not respond to voltage select!\n",mmc->control_num);
				mmcinfo("***SD/MMC %d init error!!!***\n",mmc->control_num);
				return UNUSABLE_ERR;
			}
		}
	}
#endif
	err = mmc_startup(mmc);
	if (err){
		mmcinfo("***SD/MMC %d init error!!!***\n",mmc->control_num);
		mmc->has_init = 0;
	}
	else{
		mmc->has_init = 1;
		mmcinfo("***SD/MMC %d init OK!!!***\n",mmc->control_num);
	}

	return err;
}
/*
int mmc_register(int dev_num, struct mmc *mmc)
{
	mmc_devices[dev_num] = mmc;

	if (!mmc->b_max)
		mmc->b_max = CONFIG_SYS_MMC_MAX_BLK_COUNT;

	return mmc_init(mmc);
}

int mmc_unregister(int dev_num)
{
	mmc_devices[dev_num] = NULL;
	mmcdbg("mmc%d unregister\n",dev_num);
	return 0;
}
*/




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
/*
void delay_us(unsigned n)
{
	int i = n;
	while(i--) {
		n++;
	}
}
*/
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
	//write_image_to_emmc();

	return 0;
}

