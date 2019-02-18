/*
 * QEMU model of the FTNANDC021 NAND Flash Controller
 *
 * Copyright (C) 2012 Faraday Technology
 * Written by Dante Su <dantesu@faraday-tech.com>
 *
 * This file is licensed under GNU GPL v2+.
 */

#ifndef HW_MIPS_ATHNAND_H
#define HW_MIPS_ATHNAND_H

#include "qemu/bitops.h"

/* NANDC control registers */
#define REG_SR                  0x08    /* Status Register */
#define REG_ACR                 0x00    /* Access Control Register */
#define REG_FCR                 0x108    /* Flow Control Register */
#define REG_MCR                 0x84    /* Memory Configuration Register */
#define REG_ATR1                0x80   /* AC Timing Register 1 */
#define REG_ATR2                0x90    /* AC Timing Register 2 */
#define REG_IDRL                0x120    /* ID Register LSB */
#define REG_IDRH                0x124    /* ID Register MSB */
#define REG_IER                 0x10    /* Interrupt Enable Register */
#define REG_ISCR                0xC    /* Interrupt Status Clear Register */
#define REG_WRBR                0x140    /* Write Bad Block Register */
#define REG_WRLSN               0x144    /* Write LSN Register */
#define REG_WRCRC               0x148    /* Write LSN CRC Register */
#define REG_RDBR                0x150    /* Read Bad Block Register */
#define REG_RDLSN               0x154    /* Read LSN Register */
#define REG_RDCRC               0x158    /* Read LSN CRC Register */

/* BMC control registers */
#define REG_PRR                 0x208    /* BMC PIO Ready Register */
#define REG_BCR                 0x20C    /* BMC Burst Control Register */

/** MISC register **/
#define REG_DR                  0x300    /* Data Register */
#define REG_PCR                 0x308    /* Page Count Register */
#define REG_RSTR                0x30C    /* MLC Reset Register */
#define REG_REVR                0x3F8    /* Revision Register */
//#define REG_CFGR                0x04    /* Configuration Register */


/*
 * Register BITMASK
 */
#define SR_BLANK                BIT(7)  /* blanking check failed */
#define SR_ECC                  BIT(6)  /* ecc failed */
#define SR_STS                  BIT(4)  /* status error */
#define SR_CRC                  BIT(3)  /* crc error */
#define SR_CMD                  BIT(1)  /* command finished */
#define SR_BUSY                 BIT(2)  /* chip busy */
#define SR_ENA                  BIT(0)  /* chip enabled */

#define ACR_CMD(x)              (((x) & 0x1f) << 8) /* command code */
#define ACR_START               BIT(7)  /* command start */

#define FCR_16BIT               BIT(4)  /* 16 bit data bus */
#define FCR_WPROT               BIT(3)  /* write protected */
#define FCR_NOSC                BIT(2)  /* bypass status check error */
#define FCR_MICRON              BIT(1)  /* Micron 2-plane command */
#define FCR_NOBC                BIT(0)  /* skip blanking check error */

#define IER_ENA                 BIT(7)  /* interrupt enabled */
#define IER_ECC                 BIT(3)  /* ecc error timeout */
#define IER_STS                 BIT(2)  /* status error */
#define IER_CRC                 BIT(1)  /* crc error */
#define IER_CMD                 BIT(1)  /* command finished */

/*
 * FTNANDC021 integrated command set
 */
#define ATHNAND_CMD_RESET    0x00
#define ATHNAND_CMD_RDID     0x21    /* read id */
#define ATHNAND_CMD_RDPARAM  0x22    /* read parameter page */
#define ATHNAND_CMD_RDST     0x24    /* read status */
#define ATHNAND_CMD_UNK      0x27
#define ATHNAND_CMD_RDPG     0x2a    /* read page (data + oob) */
#define ATHNAND_CMD_RDOOB    0x06    /* read oob */
#define ATHNAND_CMD_WRPG     0x0c    /* write page (data + oob) */
#define ATHNAND_CMD_ERBLK    0x0e    /* erase block */
#define ATHNAND_CMD_WROOB    0x13    /* write oob */

#endif
