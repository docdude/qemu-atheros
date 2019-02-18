/*
 * QEMU model of the  Atheros NAND Flash Controller
 *
 * Code modified 
 * Originally written by Dante Su <dantesu@faraday-tech.com>
 * Rewritten/Migrated by Juan Loya, MD <jloyamd@me.com> 
 *
 *
 */
#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu-common.h"
#include "hw/sysbus.h"
#include "hw/devices.h"
#include "hw/block/flash.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"
//#include "nand.h"
#include "ath_nand.h"

//#define DEBUG_NAND
#ifdef DEBUG_NAND
#define DB_PRINT(...) do { \
    fprintf(stderr,  "[**%s**] ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif
#define TYPE_ATHNAND "ath_nand"


#define KSEG0_BASE 0x80000000UL
#define KSEG1_BASE 0xA0000000UL
#define KSEG2_BASE 0xC0000000UL
#define KSEG3_BASE 0xE0000000UL

#define __virt_to_phys(x) \
       ( ((x) < KSEG0_BASE) ? \
         ((x)): \
         (((x) >= KSEG0_BASE) && ((x) < KSEG1_BASE)) ? \
         ((x) - KSEG0_BASE): \
         ((x) - KSEG1_BASE) )

#define KSEG1_virt_to_phys(x) \
       ( ((x) < KSEG1_BASE) ? \
         ((x)): \
         ((x) - KSEG1_BASE) )

typedef struct AthnandState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion mmio;

    qemu_irq irq;
    DeviceState *flash;

    /* DMA hardware handshake */
    qemu_irq req;

    uint8_t  manf_id, chip_id;

    int      cmd;
    int      len;    /* buffer length for page read/write */
    int      pi;    /* page index */
    int      bw;    /* bus width (8-bits, 16-bits) */
    int      addr_sel;  /* address register select */
    int      input_sel; /* input module-- SIU(0) DMA(1) */
    uint32_t fifo_buf;

    uint64_t size;    /* flash size (maximum access range) */
    uint32_t pgsz;    /* page size (Bytes) */
    uint32_t bksz;    /* block size (Bytes) */
    uint32_t sasz;    /* spare area size (Bytes) */
    uint32_t alen;    /* address length (cycle) */

    uint32_t id[2];
    uint8_t  oob[8];/* 5 bytes for 512/2048 page; 7 bytes for 4096 page */

    /* HW register caches */
    uint32_t sr;
    uint32_t fcr;
//    uint32_t mcr;
    uint32_t cfgr;
//    uint32_t ier;
    uint32_t nf_int_mask;
    uint32_t nf_int_status;
    uint32_t bcr;
    uint32_t nf_pg_size;
    uint32_t nf_time_asyn;
    uint32_t nf_time_syn;
    uint32_t nf_time_seq;
    uint32_t nf_addr0_0;  //column
    uint32_t nf_addr0_1;  //row

    uint32_t nf_dma_addr;
    uint32_t nf_dma_addr_offset;
    uint32_t nf_dma_count;
    uint32_t nf_dma_ctrl;

    uint32_t nf_rd_status;
    uint32_t nf_ecc_offset;
    uint32_t nf_ecc_ctrl;
    uint32_t nf_mem_ctrl;
    uint32_t nf_fifo_init;
    uint32_t nf_fifo_data;  
    uint32_t nf_fifo_status;
    uint32_t nf_gen_seq_ctrl;

} AthnandState;

#define ATHNAND(obj) \
    OBJECT_CHECK(AthnandState, (obj), TYPE_ATHNAND)

static void ath_nand_update_irq(AthnandState *s)
{
    if (s->nf_int_status & IER_ENA) {
        if ((s->nf_int_status & 0x0f) & (s->sr >> 2)) {
            qemu_set_irq(s->irq, 1);
        } else {
            qemu_set_irq(s->irq, 0);
        }
    }
}

static void ath_nand_set_idle(AthnandState *s)
{
    /* CLE=0, ALE=0, CE=1 */
    nand_setpins(s->flash, 0, 0, 1, 1, 0);

    /* Set command compelete */
    s->nf_int_mask |= ATH79_NF_CMD_END_INT;
//    s->nf_int_status |= ATH79_NF_CMD_END_INT;
    /* Update IRQ signal */
    s->nf_fifo_init = 0x0;
    ath_nand_update_irq(s);
}

static void ath_nand_set_cmd(AthnandState *s, uint8_t cmd)
{
    /* CLE=1, ALE=0, CE=0 */
    nand_setpins(s->flash, 1, 0, 0, 1, 0);

    /* Write out command code */
    nand_setio(s->flash, cmd);
}

static void ath_nand_set_addr(AthnandState *s, int col, int row)
{
    /* CLE=0, ALE=1, CE=0 */
    nand_setpins(s->flash, 0, 1, 0, 1, 0);

    if ((col < 0 && row < 0)|| row == 0x20) {
        /* special case for READ_ID (0x90) & READ_PARAMPAGE (0xEC) */
        if (row == 0x20) {
           nand_setio(s->flash, 0x20);
        } else {
           nand_setio(s->flash, 0);
        }
    } else {
        /* column address */
        if (col >= 0) {
            nand_setio(s->flash, extract32(col, 0, 8));
            nand_setio(s->flash, extract32(col, 8, 8));
        }
        /* row address */
        if (row >= 0) {
            nand_setio(s->flash, extract32(row, 0, 8));
            if (s->alen >= 4) {
                nand_setio(s->flash, extract32(row, 8, 8));
            }
            if (s->alen >= 5) {
                nand_setio(s->flash, extract32(row, 16, 8));
            }
        }
    }
}

static void ath_nand_handle_ack(void *opaque, int line, int level)
{
    AthnandState *s = ATHNAND(opaque);

    if (!s->bcr) {
        return;
    }

    if (level) {
        qemu_set_irq(s->req, 0);
    } else if (s->len > 0) {
        qemu_set_irq(s->req, 1);
    }
}

static void ath_nand_command(AthnandState *s, uint32_t cmd)
{
    int i,j;
    uint32_t dma_addr = 0;
    uint8_t buf = 0;

    s->nf_int_mask &= ~IER_CMD;
    s->cmd = cmd;
    DB_PRINT("cmd_seq=0x%02x\n", s->cmd);  //TODO: Add command extraction from controller command sequence
    DB_PRINT("%s selected, %s selected\n",(s->input_sel) ? "DMA" : "SIU", (s->addr_sel) ? "Address_1 Register" : "Address_0 Register"); 
    switch (cmd) {
    case  ATHNAND_CMD_RDID:    /* read id */
        ath_nand_set_cmd(s, 0x90);
        if (s->nf_addr0_0 == 0x20) { /* read ONFI signature */
           ath_nand_set_addr(s, -1, s->nf_addr0_0);
        } else {
           ath_nand_set_addr(s, -1, -1);
        }
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->len = s->nf_pg_size;
                   DB_PRINT("command=0x%02x len=0x%x \n", s->cmd, s->nf_pg_size - s->len);
        /* Set Input to SIU or DMA */
               
        if (!s->input_sel) {  /* SIU selected */
          break;
#if 0
        if (s->nf_pg_size == 8) { //TODO: Fix should be s->bw (iowidth size)
            s->id[0] = (nand_getio(s->flash) << 0)
                     | (nand_getio(s->flash) << 8)
                     | (nand_getio(s->flash) << 16)
                     | (nand_getio(s->flash) << 24);
            s->id[1] = (nand_getio(s->flash) << 0);
        } else {
            s->id[0] = (nand_getio(s->flash) << 8)
                     | (nand_getio(s->flash) << 16);
            s->id[1] = (nand_getio(s->flash) << 8);
        } 

            memcpy(&s->nf_fifo_data,s->id,4);
            memcpy(&s->nf_fifo_data+4,s->id+1,4);
#endif
        } else {  /* DMA selected */
#if 1
        if (s->nf_pg_size == 8) { //TODO: Fix should be s->bw (iowidth size)
            s->id[0] = (nand_getio(s->flash) << 24)
                     | (nand_getio(s->flash) << 16)
                     | (nand_getio(s->flash) << 8)
                     | (nand_getio(s->flash) << 0);
            s->id[1] = (nand_getio(s->flash) << 0);
        } else {
            s->id[0] = (nand_getio(s->flash) << 16)
                     | (nand_getio(s->flash) << 8);
            s->id[1] = (nand_getio(s->flash) << 16);
        } 
#endif
            dma_addr = (uint32_t)__virt_to_phys(s->nf_dma_addr);
            DB_PRINT("++++++++++dma_addr 0x%x\n",dma_addr);
            s->len = s->nf_pg_size;

            address_space_write(&address_space_memory, dma_addr,
                        MEMTXATTRS_UNSPECIFIED, (uint8_t *)&s->id,5);

//            cpu_physical_memory_write(s->nf_dma_addr, s->id, s->nf_dma_count );
            address_space_write(&address_space_memory, dma_addr+4,
                        MEMTXATTRS_UNSPECIFIED, (uint8_t *)&s->id+1,4);   
         }


#if 1
        /* page size */
        s->pgsz = (1024 << extract32(s->id[0], 0, 2));
        /* block size */
        s->bksz = (1 << (6 + extract32(s->id[0], 4, 2)));
        /* spare area size */
        s->sasz = (8 << extract32(s->id[0], 2, 1));
        /* io width */
        s->bw = (8 << extract32(s->id[0], 6, 1));

        DB_PRINT("pgsz = %d, bksz = %d, sasz = %d, bw = %d\n",s->pgsz, s->bksz, s->sasz, s->bw); 
#endif
        break;
    case  ATHNAND_CMD_RDPARAM: /* read parameter page */
        ath_nand_set_cmd(s, 0xEC);
        ath_nand_set_addr(s, -1, -1);
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->len = s->nf_pg_size;
        if (!s->input_sel) {  /* SIU selected */

 

        } else {  /* DMA selected */
           dma_addr = (uint32_t)__virt_to_phys(s->nf_dma_addr);
   //      DB_PRINT("++++++++++dma_addr 0x%x\n",dma_addr);

           for (i = 0; i < s->nf_pg_size && s->len > 0; i+=4) {

               for (j = 3; j >= 0; j--, s->len--) {  // Big Endian
                   buf = (uint8_t)nand_getio(s->flash);
                   DB_PRINT("command=0x%02x len=0x%x buf=0x%x\n", s->cmd, s->nf_pg_size - s->len, buf);
                    address_space_write(&address_space_memory, dma_addr + i + j,
                             MEMTXATTRS_UNSPECIFIED, &buf, 1);
               }
           }
        }
        break;
    case  ATHNAND_CMD_RESET:    /* reset */
        ath_nand_set_cmd(s, 0xff);
        break;
    case  ATHNAND_CMD_RDST:    /* read status */
        ath_nand_set_cmd(s, 0x70);
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->id[1] = (nand_getio(s->flash) << 0);
        break;
    case  ATHNAND_CMD_RDPG:    /* read page */
           if (s->nf_pg_size == 0x40) {
            s->nf_addr0_1 =(s->nf_addr0_1 << 16) + 0x800;
           } else {  s->nf_addr0_1 = s->nf_addr0_1 << 16; }
        ath_nand_set_cmd(s, 0x00);
      //ath_nand_set_addr(s,     int col,           int row     )
        ath_nand_set_addr(s, s->nf_addr0_1 , s->nf_addr0_0 >> 16);
        ath_nand_set_cmd(s, 0x30);
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->len = s->nf_pg_size;
        if (!s->input_sel) {  // SIU selected

        } else {   // DMA selected
           dma_addr = (uint32_t)__virt_to_phys(s->nf_dma_addr);
           DB_PRINT("++++++++++dma_addr 0x%x\n",dma_addr);
           if (s->bw == 8) {
               for (i = 0; i < s->nf_pg_size && s->len > 0; i++, s->len--) {
                   buf = (uint8_t)nand_getio(s->flash);
                 // buf = ret;

                   DB_PRINT("command=0x%02x len=0x%x buf=0x%x\n", s->cmd, s->nf_pg_size - s->len, buf);

                   address_space_write(&address_space_memory, dma_addr + i,
                               MEMTXATTRS_UNSPECIFIED, &buf, 1);
                   //cpu_physical_memory_write(s->nf_dma_addr, &ret, s->nf_dma_count );
                }

           } else {
               for (i = 0; i < 2 && s->len > 1; i++, s->len -= 2) {
                       buf = deposit32(buf, i * 16, 16, nand_getio(s->flash));
               }
           }
        }
        break;
    case  ATHNAND_CMD_RDOOB:    /* read oob */
        ath_nand_set_cmd(s, 0x00);
        ath_nand_set_addr(s, s->nf_addr0_1 << 16, s->nf_addr0_0 >> 16);
        ath_nand_set_cmd(s, 0x30);
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->len = s->nf_pg_size;
        if (!s->input_sel) {  // SIU selected

        } else {   // DMA selected


        for (i = 0; i < s->nf_pg_size;) {
            if (s->bw == 8) {
             //   if (i > 16 && i < 32) {
                    s->oob[i] = (uint8_t)nand_getio(s->flash);
           //     } else {
              //      (void)nand_getio(s->flash);
             //   }

 //                   DB_PRINT("command=0x%02x len=0x%x buf=0x%x\n", s->cmd, 0x840 - s->len, buf);
//DB_PRINT("++++++++++dma_addr 0x%x\n",dma_addr);
                    address_space_write(&address_space_memory, dma_addr+0x800 + i,
                                MEMTXATTRS_UNSPECIFIED, &s->oob[i], 1);
                i += 1;
            } else {
                if (i < 16) {
                    *(uint16_t *)(s->oob + i) = (uint16_t)nand_getio(s->flash);
                } else {
                    (void)nand_getio(s->flash);
                }
                i += 2;
            }
        }
        memcpy(&s->nf_fifo_data,s->oob,s->nf_pg_size);
    }
        break;
    case  ATHNAND_CMD_WRPG:    /* write page + read status */
        ath_nand_set_cmd(s, 0x80);
        ath_nand_set_addr(s, 0, s->pi);
        /* data phase */
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        s->len = s->nf_pg_size;
        break;
    case  ATHNAND_CMD_ERBLK:    /* erase block + read status */
        ath_nand_set_cmd(s, 0x60);
        ath_nand_set_addr(s, -1, s->pi);
        ath_nand_set_cmd(s, 0xd0);
        /* read status */
        ath_nand_command(s, 0x04);
        break;
    case  ATHNAND_CMD_WROOB:    /* write oob + read status */
        ath_nand_set_cmd(s, 0x80);
        ath_nand_set_addr(s, s->nf_pg_size, s->pi);
        /* data phase */
        nand_setpins(s->flash, 0, 0, 0, 1, 0);
        for (i = 0; i < 16 * (s->nf_pg_size / 512); ) {
            if (s->bw == 8) {
                if (i <= 7) {
                    nand_setio(s->flash, s->oob[i]);
                } else {
                    nand_setio(s->flash, 0xffffffff);
                }
                i += 1;
            } else {
                if (i <= 7) {
                    nand_setio(s->flash, s->oob[i] | (s->oob[i + 1] << 8));
                } else {
                    nand_setio(s->flash, 0xffffffff);
                }
                i += 2;
            }
        }
        ath_nand_set_cmd(s, 0x10);
        /* read status */
        ath_nand_command(s, 0x04);
        break;
    default:
        DB_PRINT("unknown command=0x%02x\n", cmd);
        break;
    }

    /* if cmd is not page read/write, then return to idle mode */
    switch (s->cmd) {
    case  ATHNAND_CMD_RDPG:
    case  ATHNAND_CMD_WRPG:
    case  ATHNAND_CMD_RDPARAM:
    case  ATHNAND_CMD_RDID:
        if (s->bcr && (s->len > 0)) {
            qemu_set_irq(s->req, 1);
        }

        break;
    default:
        ath_nand_set_idle(s);
        break;
    }
    /* command completed */
    s->nf_int_status |= ATH79_NF_CMD_END_INT;


}

static uint64_t
ath_nand_mem_read(void *opaque, hwaddr addr, unsigned size)
{
    uint32_t  ret = 0;
    int i=0;
    AthnandState *s = ATHNAND(opaque);

//DB_PRINT("memory access@%#" HWADDR_PRIx "\n", addr);
    switch (addr) {

    /* #define ATH_NF_CTRL	       (ATH_NAND_FLASH_BASE + 0x04u) */
    case ATH79_NF_CTRL:
        ret =  s->cfgr;
        break;
    /* #define ATH79_NF_STATUS		(ATH_NAND_FLASH_BASE + 0x008u) */
    case REG_SR:
        ret = s->sr=0xff;
	break;
    /* #define ATH79_NF_CTRL		(ATH_NAND_FLASH_BASE + 0x004u) */
    case ATH79_NF_COMMAND:
        ret = s->cmd;
        break;
    case REG_RDBR:
        ret = s->oob[0];
	break;
    case REG_RDLSN:
        ret = s->oob[1] | (s->oob[2] << 8);
        break;
    case REG_RDCRC:
        if (s->pgsz > 2048) {
            ret = s->oob[3] | (s->oob[4] << 8)
                   | (s->oob[5] << 16) | (s->oob[6] << 24);
        } else {
            ret = s->oob[3] | (s->oob[4] << 8);
        }
        break;
    /* #define ATH79_NF_INT_STATUS	(ATH_NAND_FLASH_BASE + 0x010u) */
    case ATH79_NF_INT_STATUS:
        ret = s->nf_int_status;
	break;
    /* #define ATH79_NF_INT_MASK	(ATH_NAND_FLASH_BASE + 0x00cu) */
    case ATH79_NF_INT_MASK:
        ret = s->nf_int_mask;
	break;

     /* #define ATH_NF_ECC_CTRL		(ATH_NAND_FLASH_BASE + 0x014u) */
    case ATH79_NF_ECC_CTRL:
       ret = s->nf_ecc_ctrl;
	break;
    /* #define ATH_NF_ECC_OFFSET	(ATH_NAND_FLASH_BASE + 0x018u) */
    case ATH79_NF_ECC_OFFSET:
       ret = s->nf_ecc_offset;
	break;
    /* #define ATH_NF_ADDR0_0		(ATH_NAND_FLASH_BASE + 0x01cu) */
    case ATH79_NF_ADDR0_0:
       ret = s->nf_addr0_0;
	break;

    /* #define ATH_NF_ADDR0_1		(ATH_NAND_FLASH_BASE + 0x024u) */
    case ATH79_NF_ADDR0_1:
        ret = s->nf_addr0_1;
	break;

    /* #define ATH_NF_DMA_ADDR		(ATH_NAND_FLASH_BASE + 0x064u) */
    case ATH79_NF_DMA_ADDR:
        ret = s->nf_dma_addr;
	break;
    /* #define ATH_NF_DMA_COUNT	        (ATH_NAND_FLASH_BASE + 0x068u) */
    case ATH79_NF_DMA_COUNT:
        ret = s->nf_dma_count;
	break;

    /* #define ATH_NF_DMA_CTRL		(ATH_NAND_FLASH_BASE + 0x06cu) */
    case ATH79_NF_DMA_CTRL:
        if (s->len <= 0) {
           ath_nand_set_idle(s);
        }
        ret = s->nf_dma_ctrl=0x4d;
	break;
    /* #define ATH_NF_MEM_CTRL		(ATH_NAND_FLASH_BASE + 0x080u) */
    case ATH79_NF_MEM_CTRL:
        ret = s->nf_mem_ctrl;
	break;
    /* #define ATH_NF_PG_SIZE		(ATH_NAND_FLASH_BASE + 0x084u) */
    case ATH79_NF_PG_SIZE:
        ret = s->nf_pg_size;
	break;
    /* #define ATH_NF_RD_STATUS	        (ATH_NAND_FLASH_BASE + 0x088u) */
     case ATH79_NF_RD_STATUS:
        ret = s->nf_rd_status = 0xc0;  //hack==========================================
	break;
    /* #define ATH79_NF_TIME_SEQ	(ATH_NAND_FLASH_BASE + 0x08cu) */
    case ATH79_NF_TIME_SEQ:
        ret = s->nf_time_seq;
	break;
    /* #define ATH79_NF_TIMINGS_ASYN	(ATH_NAND_FLASH_BASE + 0x090u) */
    case ATH79_NF_TIMINGS_ASYN:
        ret = s->nf_time_asyn;
	break;  /* AC Timing */
    /* #define ATH79_NF_TIMINGS_SYN	(ATH_NAND_FLASH_BASE + 0x094u) */
    case ATH79_NF_TIMINGS_SYN:
        ret = s->nf_time_syn;
	break;
    /* #define ATH79_NF_FIFO_DATA	(ATH_NAND_FLASH_BASE + 0x098u) */
    case ATH79_NF_FIFO_DATA:
//        cpu_physical_memory_write(0xc0000000, &s->nf_fifo_data, s->nf_dma_count );  //hack to crash linux at nand code
//        ret = (intptr_t)s->nf_fifo_data;
//           for (i=0; i < s->nf_pg_size; i++) {


//printf("param data = 0x%x\n",s->nf_fifo_data[i]);
 //          }
 //       memcpy(&ret,s->nf_fifo_data,8);
#if 0
          for (i=0; i <  4 && s->len > 0; i++, s->len--) {
 //             buf = (uint8_t)nand_getio(s->flash);
                s->nf_fifo_data[i] = (nand_getio(s->flash) << 0)
                         | (nand_getio(s->flash) << 8)
                         | (nand_getio(s->flash) << 16)
                         | (nand_getio(s->flash) << 24);
//            s->nf_fifo_data[i] = (uint8_t)nand_getio(s->flash);

           }
#else
                  DB_PRINT("command=0x%02x len=0x%x ret=0x%x\n", s->cmd, s->nf_pg_size - s->len, ret);
        if ((s->nf_fifo_init) && (s->len > 0)) {
            if (s->bw == 8) {
                for (i = 0; i < 4 && s->len > 0; i++, s->len--) {
                    s->nf_fifo_data = deposit32(s->nf_fifo_data, i * 8, 8, nand_getio(s->flash));
                }
            } else {
                for (i = 0; i < 2 && s->len > 1; i++, s->len -= 2) {
                    s->nf_fifo_data = deposit32(s->nf_fifo_data, i * 16, 16, nand_getio(s->flash));
                }
            }
            if (s->len <= 0) {
                ath_nand_set_idle(s);
            }
        }
 

#endif
        ret = s->nf_fifo_data;
	break;
    /* #define ATH79_NF_DMA_ADDR_OFFSET	(ATH_NAND_FLASH_BASE + 0x0a0u) */
    case ATH79_NF_DMA_ADDR_OFFSET:
        ret = s->nf_dma_addr_offset;
	break;
    /* #define ATH_NF_FIFO_INIT	        (ATH_NAND_FLASH_BASE + 0x0b0u) */
    case ATH79_NF_FIFO_INIT:
        ret = s->nf_fifo_init;
	break;
 
    /* #define ATH_NF_GENERIC_SEQ_CTRL	(ATH_NAND_FLASH_BASE + 0x0b4u) */
    case ATH79_NF_GENERIC_SEQ_CTRL:
        ret = s->nf_gen_seq_ctrl;
	break;
    /* #define ATH79_NF_FIFO_STATUS	(ATH_NAND_FLASH_BASE + 0x0b8u) */
    case ATH79_NF_FIFO_STATUS:
        ret = s->nf_fifo_status;
	break;
    default:
        DB_PRINT("undefined memory access@%#" HWADDR_PRIx "\n", addr);
        break;
    }

    DB_PRINT("memory access@%#" HWADDR_PRIx " val=0x%x\n", addr, ret);
    return ret;
}

static void
ath_nand_mem_write(void *opaque, hwaddr addr, uint64_t val, unsigned size)
{

    AthnandState *s = ATHNAND(opaque);
    DB_PRINT("memory access@%#" HWADDR_PRIx " val=0x%lx\n", addr, val);
    switch (addr) {
    /* #define ATH79_NF_CTRL	(ATH_NAND_FLASH_BASE + 0x004u) */
    case ATH79_NF_COMMAND:
    //    if (!(val & ACR_START)) {
      //      break;
       // }
        s->addr_sel = extract32((uint32_t)val, 7, 1);
        s->input_sel = extract32((uint32_t)val, 6, 1);
        ath_nand_command(s, extract32((uint32_t)val, 0, 6));

        break;
    /* #define ATH_NF_CTRL	(ATH_NAND_FLASH_BASE + 0x04u) */
    case ATH79_NF_CTRL:
        s->cfgr = (uint32_t)val;
#if 1
        /* page size */
        switch (extract32(s->cfgr, 8, 3)) {
        case 0:
            s->pgsz = 256;
            break;
        case 1:
            s->pgsz = 512;
            break;
        case 2:
            s->pgsz = 1024;
            break;
        case 3:
            s->pgsz = 2048;
            break;
        case 4:
            s->pgsz = 4096;
            break;

        default:
            s->pgsz = 2048;
            break;
        }
        /* block size */
        s->bksz = (1 << (5 + extract32(s->cfgr, 6, 2)));
        /* address length (cycle) */
        s->alen = extract32(s->cfgr, 0, 3);
        /* flash size */
        s->size = 1ULL << (24 + extract32(s->cfgr, 4, 4));
        /* io width */
        s->bw = (8 << extract32(s->cfgr, 12, 1));
#endif
DB_PRINT("cfgr = 0x%x, pgsz = %d, bksz = %d, alen = %d, bw = %d\n",s->cfgr, s->pgsz, s->bksz, s->alen, s->bw); 
 /* 5 address-cycles, spare area enabled, glocal interrupt disabled, ECC enabled, 64 pages/block, 2048 page size, 8-bit BCH, 16 bit flash, 1 chip */
        break;
    case REG_WRBR:
        s->oob[0] = (uint32_t)val & 0xff;
        break;
    case REG_WRLSN:
        s->oob[1] = ((uint32_t)val >> 0) & 0xff;
        s->oob[2] = ((uint32_t)val >> 8) & 0xff;
        break;
    case REG_WRCRC:
        s->oob[3] = ((uint32_t)val >> 0) & 0xff;
        s->oob[4] = ((uint32_t)val >> 8) & 0xff;
        if (s->pgsz > 2048) {
            s->oob[5] = ((uint32_t)val >> 16) & 0xff;
            s->oob[6] = ((uint32_t)val >> 24) & 0xff;
        }
        break;
    case REG_FCR:
        s->fcr = (uint32_t)val;
        if (s->fcr & FCR_16BIT) {
            s->bw = 16;
        } else {
            s->bw = 8;
        }
        break;
    /* #define ATH79_NF_INT_STATUS	(ATH_NAND_FLASH_BASE + 0x010u) */
    case REG_IER:
        s->nf_int_status = val;
        ath_nand_update_irq(s);
        break;
    /* #define ATH79_NF_INT_MASK	(ATH_NAND_FLASH_BASE + 0x00cu) */
    case ATH79_NF_INT_MASK:
        s->sr = val;//&= ~(((uint32_t)val & 0x0f) << 2);
        ath_nand_update_irq(s);
        break;
    /* #define ATH_NF_ECC_CTRL		(ATH_NAND_FLASH_BASE + 0x014u) */
    case ATH79_NF_ECC_CTRL:
        s->nf_ecc_ctrl = val;
        break;
    /* #define ATH_NF_ECC_OFFSET	(ATH_NAND_FLASH_BASE + 0x018u) */
    case ATH79_NF_ECC_OFFSET:
        s->nf_ecc_offset = val;
        break;
    /* #define ATH_NF_ADDR0_0		(ATH_NAND_FLASH_BASE + 0x01cu) */
    case ATH79_NF_ADDR0_0:
        s->nf_addr0_0 = (uint32_t)val;
        s->pi = s->nf_addr0_0;
        break;
    /* #define ATH_NF_ADDR0_1		(ATH_NAND_FLASH_BASE + 0x024u) */
    case ATH79_NF_ADDR0_1:
        s->nf_addr0_1 = (uint32_t)val;

        break;
    /* #define ATH_NF_DMA_ADDR		(ATH_NAND_FLASH_BASE + 0x064u) */
    case ATH79_NF_DMA_ADDR:
        s->nf_dma_addr = val;

        break;
    /* #define ATH_NF_DMA_COUNT 	(ATH_NAND_FLASH_BASE + 0x068u) */
    case ATH79_NF_DMA_COUNT:
        s->nf_dma_count = val;

        break;
    /* #define ATH_NF_DMA_CTRL		(ATH_NAND_FLASH_BASE + 0x06cu) */
    case ATH79_NF_DMA_CTRL:
        s->nf_dma_ctrl = val;

        break;
    /* #define ATH_NF_MEM_CTRL		(ATH_NAND_FLASH_BASE + 0x080u) */
    case ATH79_NF_MEM_CTRL:
        s->nf_mem_ctrl = val;
        break; 
    /* #define ATH_NF_PG_SIZE		(ATH_NAND_FLASH_BASE + 0x084u) */
    case ATH79_NF_PG_SIZE:
        s->nf_pg_size = (uint32_t)val;
        break;
    /* #define ATH79_NF_TIME_SEQ	(ATH_NAND_FLASH_BASE + 0x08cu) */
    case ATH79_NF_TIME_SEQ:
        s->nf_time_seq = val;
        break;
    /* #define ATH79_NF_TIMINGS_ASYN	(ATH_NAND_FLASH_BASE + 0x090u) */
    case ATH79_NF_TIMINGS_ASYN:
        s->nf_time_asyn = val;  /* AC Timing */
        break;
    /* #define ATH79_NF_TIMINGS_SYN	(ATH_NAND_FLASH_BASE + 0x094u) */
    case ATH79_NF_TIMINGS_SYN:
        s->nf_time_syn = val;
        break;
    /* #define ATH79_NF_FIFO_DATA	(ATH_NAND_FLASH_BASE + 0x098u) */
    case ATH79_NF_FIFO_DATA:

 //       s->nf_fifo_data = val;
	break;
    /* #define ATH79_NF_DMA_ADDR_OFFSET	(ATH_NAND_FLASH_BASE + 0x0a0u) */
    case ATH79_NF_DMA_ADDR_OFFSET:
        s->nf_dma_addr_offset = val;
        break;  
    /* #define ATH_NF_FIFO_INIT 	(ATH_NAND_FLASH_BASE + 0x0b0u) */
    case ATH79_NF_FIFO_INIT:
        s->nf_fifo_init = val;
        if (s->nf_fifo_init == 0x1) {
           s->nf_fifo_status = 0x1;

        }
        break; 
   /* #define ATH_NF_GENERIC_SEQ_CTRL	(ATH_NAND_FLASH_BASE + 0x0b4u) */
    case ATH79_NF_GENERIC_SEQ_CTRL:
        s->nf_gen_seq_ctrl = val;
        break;
    /* #define ATH79_NF_FIFO_STATUS	(ATH_NAND_FLASH_BASE + 0x0b8u) */
    case ATH79_NF_FIFO_STATUS:
        s->nf_fifo_status = val;
	break;

    default:
        DB_PRINT("undefined memory access@%#" HWADDR_PRIx "\n", addr);
        break;
    }
}

static const MemoryRegionOps mmio_ops = {
    .read  = ath_nand_mem_read,
    .write = ath_nand_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

static void ath_nand_reset(DeviceState *ds)
{
    AthnandState *s = ATHNAND(SYS_BUS_DEVICE(ds));
    Error *local_errp = NULL;

    s->flash = DEVICE(object_property_get_link(OBJECT(s),
                                               "flash",
                                               &local_errp));
    if (local_errp) {
        fprintf(stderr, "ath_nand: Unable to get flash link\n");
        abort();
    }

    s->sr    = 0;
    s->nf_int_status   = 0;
    s->bcr   = 0;
    s->id[0] = 0;
    s->id[1] = 0;

    /* We can assume our GPIO outputs have been wired up now */
    qemu_set_irq(s->req, 0);
}

static void ath_nand_realize(DeviceState *dev, Error **errp)
{

    AthnandState *s = ATHNAND(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mmio,
			  OBJECT(s),
                          &mmio_ops,
                          s,
                          TYPE_ATHNAND,
                          0x1000);
    sysbus_init_mmio(sbd, &s->mmio);
    sysbus_init_irq(sbd, &s->irq);

    qdev_init_gpio_in(dev, ath_nand_handle_ack, 1);
    qdev_init_gpio_out(dev, &s->req, 1);
}

static const VMStateDescription vmstate_ath_nand = {
    .name = TYPE_ATHNAND,
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(sr, AthnandState),
        VMSTATE_UINT32(nf_int_status, AthnandState),
        VMSTATE_END_OF_LIST()
    }
};

static void ath_nand_instance_init(Object *obj)
{
    AthnandState *s = ATHNAND(obj);

    object_property_add_link(obj,
                             "flash",
                             TYPE_DEVICE,
                             (Object **) &s->flash,
                             object_property_allow_set_link,
                             OBJ_PROP_LINK_STRONG,
                             &error_abort);
}

static void ath_nand_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd    = &vmstate_ath_nand;
    dc->reset   = ath_nand_reset;
    dc->realize = ath_nand_realize;
//    dc->no_user = 1;
}

static const TypeInfo ath_nand_info = {
    .name          = TYPE_ATHNAND,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AthnandState),
    .instance_init = ath_nand_instance_init,
    .class_init    = ath_nand_class_init,
};

static void ath_nand_register_types(void)
{
    type_register_static(&ath_nand_info);
}

type_init(ath_nand_register_types)
