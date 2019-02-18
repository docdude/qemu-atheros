/*
 * QEMU ar71xx board support
 *
 * Copyright (c) 2006 Aurelien Jarno
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include "qemu/units.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/i386/pc.h"
#include "hw/isa/superio.h"
#include "hw/dma/i8257.h"
#include "hw/char/serial.h"
#include "net/net.h"
#include "hw/boards.h"
#include "hw/i2c/smbus.h"
#include "hw/block/flash.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/pci/pci.h"
#include "sysemu/sysemu.h"
#include "sysemu/arch_init.h"
#include "sysemu/block-backend.h"
#include "qemu/log.h"
#include "hw/mips/bios.h"
#include "hw/ide.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/timer/mc146818rtc.h"
#include "hw/timer/i8254.h"
#include "exec/address-spaces.h"
#include "hw/sysbus.h"             /* SysBusDevice */
#include "qemu/host-utils.h"
#include "sysemu/qtest.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "hw/empty_slot.h"
#include "sysemu/kvm.h"
#include "exec/semihost.h"
#include "hw/mips/cps.h"
#include "ath_apbuart.h"

//#define DEBUG_ERR
#ifdef DEBUG_ERR
#define DB_PRINT(...) do { \
    fprintf(stderr,  "[**%s**] ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define DEBUG_BOARD_INIT

#define ENVP_ADDR		0x80002000l
#define ENVP_NB_ENTRIES	 	16
#define ENVP_ENTRY_SIZE	 	256

/* Hardware addresses */
#define FLASH_ADDRESS 0x1f000000ULL
#define FPGA_ADDRESS  0x18000000ULL
#define RESET_ADDRESS 0x1fc00000ULL
 
#define FLASH_SIZE    0x400000

#define MAX_IDE_BUS 2

typedef struct {
    MemoryRegion iomem;
    MemoryRegion iomem_reset; 
    MemoryRegion iomem_pll; 
    MemoryRegion iomem_gmac;
    MemoryRegion iomem_wdma; 
    MemoryRegion iomem_wqcu; 
    MemoryRegion iomem_wdcu; 
    MemoryRegion iomem_rtc; 
    MemoryRegion iomem_wpcu; 
    MemoryRegion iomem_srif; 
    MemoryRegion iomem_ddr; 
    MemoryRegion iomem_gpio; 
    MemoryRegion iomem_usb; 
    MemoryRegion iomem_ge0;
    MemoryRegion iomem_ge1;
    MemoryRegion iomem_ehci;
    MemoryRegion iomem_ohci;
    MemoryRegion iomem_nfc;
    MemoryRegion iomem_spi;
    MemoryRegion iomem_unk;
    MemoryRegion iomem_pci_crp;
    MemoryRegion iomem_pci_ctrl;
    DeviceState *nand;

    uint32_t int_enable;
    uint32_t int_status;
    uint32_t wdog;
    uint32_t wdog_ctrl;
    uint32_t pll_cpu;
    uint32_t pll_ddr;
    uint32_t pll_clk_spare;  
    uint32_t reset_reg_reset;
    uint32_t gpio_out_func0;
    uint32_t gpio_out_func1;
    uint32_t gpio_out_func2;
    uint32_t gpio_out_func3;
    uint32_t gpio_out_func4;
    uint32_t gpio_out_func5;
    uint32_t gpio_out_func;
    uint32_t gpio_func;
    uint32_t gpio_out;
    uint32_t gpio_oe;
    uint32_t gpio_in_en0;

    uint32_t gpio_in_en1;
    uint32_t gpio_in_en9;
    uint32_t gpio_in;
    uint32_t ddr_tout_maxreg;
    uint32_t ddr_wmac_7040;
    uint32_t ddr_wmac_7044;
    uint32_t cpu_pll_dither;
    uint32_t srif_cpu_dpll;
    uint32_t srif_ddr_dpll;
    uint32_t srif_cpu_dpll2;
    uint32_t srif_pll_dpll2;
    uint32_t srif_unk_dpll3;
    uint32_t srif_ddr_dpll2;
    uint32_t srif_cpu_dpll3;
    uint32_t srif_ddr_dpll3;
    uint32_t srif_cpu_dpll4;
    uint32_t srif_ddr_dpll4;
    uint32_t srif_pcie_dpll4;

    uint32_t ddr_cfg;
    uint32_t ddr_cfg2;
    uint32_t ddr_mode;
    uint32_t ddr_extmode;
    uint32_t ddr_ctrl;
    uint32_t ddr_rd_data;
    uint32_t ddr_ctrl_cfg;
    uint32_t ddr_refresh;
    uint32_t ddr_tap_ctrl0;
    uint32_t ddr_tap_ctrl1;
    uint32_t ddr_tap_ctrl2;
    uint32_t ddr_tap_ctrl3;
    uint32_t ddr_perf_comp_addr_0;
    uint32_t ddr_perf_comp_addr_1;
    uint32_t ddr_perf_comp_ge0_1;
    uint32_t ddr_perf_mask_ge1_1;
    uint32_t ddr_perf_comp_ge1_1;
    uint32_t ddr_bist;
    uint32_t ddr_bist_status;
    uint32_t ddr_burst1;
    uint32_t ddr_burst2;
    uint32_t ddr_wb_flush_usb;
    uint32_t ar_phy_pmu1;
    uint32_t ar_phy_pmu2;

    SerialState *uart_ls;
    SerialState *uart_hs;

    uint32_t spi_func_sel;
    uint32_t spi_ctrl;
    uint32_t spi_io_ctrl;
    uint32_t spi_read_data;
    uint32_t spi_shift_dataout;
    uint32_t spi_shift_cnt;
    uint32_t spi_shift_datain;
    uint32_t gmac_fifo_cfg1;
    uint32_t gmac_fifo_cfg2;
    uint32_t gmac_fifo_cfg3;
    uint32_t gmac_fifo_cfg4;
    uint32_t gmac_fifo_cfg5;

} AR71xxFPGAState;

#define TYPE_MIPS_AR71XX "mips-ar71xx"
#define MIPS_AR71XX(obj) OBJECT_CHECK(AR71xxState, (obj), TYPE_MIPS_AR71XX)

typedef struct {
    SysBusDevice parent_obj;

    MIPSCPSState *cps;
    qemu_irq *i8259;
} AR71xxState;

//static ISADevice *pit;

static struct _loaderparams {
    int ram_size, ram_low_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

static uint64_t ar71xx_fpga_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t val = 0;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {
    /* #define AR934X_PLL_CPU_CONFIG_REG		0x00 */

    case 0x00:
//        val = 0x40021380;

        val = s->pll_cpu;

        break;
    /* #define AR934X_PLL_DDR_CONFIG_REG		0x04 */

    case 0x04:
//        val = 0x40212800;

        val = s->pll_ddr;

        break; 
    /* #define AR71XX_RESET_REG_WDOG_CTRL 0x08 */

    case 0x08:

        val = s->wdog_ctrl;

//#define WDOG_CTRL_LAST_RESET		BIT(31)
//#define WDOG_CTRL_ACTION_MASK		3
//#define WDOG_CTRL_ACTION_NONE		0	/* no action */
//#define WDOG_CTRL_ACTION_GPI		1	/* general purpose interrupt */
//#define WDOG_CTRL_ACTION_NMI		2	/* NMI */
//#define WDOG_CTRL_ACTION_FCR		3       /* full chip reset */

        break;
   /* #define AR71XX_RESET_REG_WDOG 0x0c */

    case 0x0c:

        val = s->wdog;

        break; 
    /* #define AR934X_GPIO_REG_OUT_FUNC5 0x40 */
    case 0x40:

        val = s->ddr_wmac_7040;


        break;
    /* #define QCA953X_GPIO_REG_IN_ENABLE0 0x44 */
    case 0x44:


        val = s->ddr_wmac_7044;

        break;
    /* #define QCA_PLL_CPU_PLL_DITHER_REG			QCA_PLL_BASE_REG + 0x48 */
    case 0x48:
        val = s->cpu_pll_dither;
        break;

    /* #define AR934X_RESET_REG_BOOTSTRAP 0xb0 */
    case 0xb0:
        val = 0x2F051B; 
        break;
    /* #define AR934X_RESET_REG_PCIE_WMAC_INT_STATUS 0xac */
    case 0xac:
        val = 0x0; 
        break;
    /* #define AR71XX_RESET_REG_MISC_INT_ENABLE	0x14 */
    case 0x14:
        val = s->int_enable;
        break;
    /* #define AR934X_RESET_REG_RESET_MODULE		0x1c */
    case 0x1c:
        val = s->reset_reg_reset;
        break;
    /* #define AR934X_SWITCH_CLOCK_SPARE             AR7240_PLL_BASE+0x0024 */
    case 0x24:
        val = s->pll_clk_spare;
        break;
    /* #define AG7240_MAC_FIFO_CFG_1      0x4c */
    case 0x4c:
        val = s->gmac_fifo_cfg1;
        break;
    /* #define AG7240_MAC_FIFO_CFG_2      0x50 */
    case 0x50:
        val = s->gmac_fifo_cfg2;
        break;
    /*#define AG7240_MAC_FIFO_CFG_3      0x54 */
    case 0x54:
        val = s->gmac_fifo_cfg3;
        break;
    /* #define AG7240_MAC_FIFO_CFG_4      0x58 */
    case 0x58:
        val = s->gmac_fifo_cfg4;
        break;
    /* #define AG7240_MAC_FIFO_CFG_5      0x5c */
    case 0x5c:
       val = s->gmac_fifo_cfg5;
        break;
    /* CHIP ID Register */
    case 0x90:
       val = 0x2123;  // 0x2120 for Atheros 934x Wasp 1.0 uBoot
       break;
    /* #define CPU_DPLL_ADDRESS                                                 0x181161c0*/
    case 0x1c0:
        val = s->srif_cpu_dpll;
        break;

    /* SRIF Register */
    case 0x1c4:
        val = s->srif_cpu_dpll2;		
        break;
    /* #define CPU_DPLL4_ADDRESS			0x181161cc */
    case 0x1cc:
        val = s->srif_cpu_dpll4;		
        break;
    /*#define DDR_DPLL4_ADDRESS			0x1811624c */
    case 0x24c:
        val = s->srif_ddr_dpll4;		
        break;
    /* #define DPLL3_ADDRESS_88			0x18116188 */
    case 0x188:
        val = s->srif_unk_dpll3;
        break;
    /* #define DPLL2_ADDRESS_44			0x18116244 */
    case 0x244:

        val = s->srif_ddr_dpll2;

        break;
    /* #define CPU_DPLL3_ADDRESS			0x181161c8 */
    case 0x1c8:
        val = s->srif_cpu_dpll3;
        break;
    /* #define DDR_DPLL3_ADDRESS			0x18116248 */
    case 0x248:

        val = s->srif_ddr_dpll3;

        break;



    /* #define DDR_DPLL_ADDRESS                                                 0x18116240 */
    case 0x240:
        val = s->srif_ddr_dpll;
        break;

    case 0xc40:
        val = s->ar_phy_pmu1;
        break;
    /* ddr ldo tune */
    case 0xc44:
        val = s->ar_phy_pmu2;
        break;
    /* #define PCIe_DPLL4_ADDRESS			0x18116c0c */
    case 0xc0c:
        val = s->srif_pcie_dpll4;
    break;

    /* WMAC Register ?? */
    case 0x7040:
        val = s->ddr_wmac_7040;
        break;
    /* WMAC Register ?? */
    case 0x7044:
        val = s->ddr_wmac_7044;
        break;
    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08x\n", addr, val);
#endif
        break;
    }
    return val;
}

static void ar71xx_fpga_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {
    /* #define AR934X_PLL_CPU_CONFIG_REG		0x00 */

    case 0x00:

        s->pll_cpu = val;

        break;
    /* #define AR934X_PLL_DDR_CONFIG_REG		0x04 */
    case 0x04:

        s->pll_ddr = val;

        break; 
    /* #define AR71XX_RESET_REG_WDOG_CTRL 0x08 */
    case 0x08:

        s->wdog_ctrl = val;

//#define WDOG_CTRL_LAST_RESET		BIT(31)
//#define WDOG_CTRL_ACTION_MASK		3
//#define WDOG_CTRL_ACTION_NONE		0	/* no action */
//#define WDOG_CTRL_ACTION_GPI		1	/* general purpose interrupt */
//#define WDOG_CTRL_ACTION_NMI		2	/* NMI */
//#define WDOG_CTRL_ACTION_FCR		3       /* full chip reset */

        break;
    /* #define AR71XX_RESET_REG_WDOG 0x0c */
    case 0x0c:
        s->wdog = val;
        break; 
    /* #define AR71XX_RESET_REG_MISC_INT_STATUS	0x10 */
    case 0x10:
        s->int_status = val;
        break;
    /* #define AR71XX_RESET_REG_MISC_INT_ENABLE	0x14 */
    case 0x14:
        s->int_enable = val;
        break;

    /* #define AR934X_RESET_REG_RESET_MODULE		0x1c */ ////dup index with QCA_DDR_TAP_CTRL_0_REG 0x1c
    case 0x1c:
     DB_PRINT ("FIXME dup register offset: 0x%08lx val: 0x%08lx\n", (uint64_t) addr &0xffffffff, val);
        s->reset_reg_reset = val;
        break;
    /* #define AR934X_SWITCH_CLOCK_SPARE             AR7240_PLL_BASE+0x0024 */
    case 0x24:
        s->pll_clk_spare = val;
        break;

    /* #define AR934X_GPIO_REG_OUT_FUNC0 0x2c */
    case 0x2c:
     DB_PRINT ("FIXME dup register offset: 0x%08lx val: 0x%08lx\n", (uint64_t) addr &0xffffffff, val);
 //       s->gpio_out_func0 = val;
        s->ddr_perf_comp_addr_0 = val;
        break;

    /* #define AR934X_GPIO_REG_OUT_FUNC5 0x40 */
    case 0x40:

        s->ddr_wmac_7040 = val;
        break;
    /* #define QCA953X_GPIO_REG_IN_ENABLE0 0x44 */
    case 0x44:

        s->ddr_wmac_7044 = 0x3;
        break;
    /* #define QCA_PLL_CPU_PLL_DITHER_REG			QCA_PLL_BASE_REG + 0x48 */
    case 0x48:
        s->cpu_pll_dither = val;
        break;
    /* #define AG7240_MAC_FIFO_CFG_1      0x4c */
    case 0x4c:
        s->gmac_fifo_cfg1 = val;
        break;
    /* #define AG7240_MAC_FIFO_CFG_2      0x50 */
    case 0x50:
        s->gmac_fifo_cfg2 = val;
        break;
    /*#define AG7240_MAC_FIFO_CFG_3      0x54 */
    case 0x54:
        s->gmac_fifo_cfg3 = val;
        break;
    /* #define AG7240_MAC_FIFO_CFG_4      0x58 */
    case 0x58:
        s->gmac_fifo_cfg4 = val;
        break;
    /* #define AG7240_MAC_FIFO_CFG_5      0x5c */
    case 0x5c:
        s->gmac_fifo_cfg5 = val;
        break;
    case 0xc40:
        s->ar_phy_pmu1 = val;
        break;
    /* ddr ldo tune */
    case 0xc44:
        s->ar_phy_pmu2 = val;
        break;
    /* WMAC Register ?? */
    case 0x7040:
        s->ddr_wmac_7040 = val;
        break;
    /* WMAC Register ?? */
    case 0x7044:
        s->ddr_wmac_7044 = val;
        break;
    /*#define CPU_DPLL_ADDRESS                                                 0x181161c0 */
    case 0x1c0:
        s->srif_cpu_dpll = val;
        break;
    /* #define DDR_DPLL_ADDRESS                                                 0x18116240 */
    case 0x240:
        s->srif_ddr_dpll = val;
        break;
    /* QCA_PLL_SRIF_CPU_DPLL2_REG == #define QCA_PLL_SRIF_CPU_DPLL_BASE_REG	QCA_PLL_SRIF_BASE_REG + 0x1C0 + 0x4 */
    case 0x1c4:
        s->srif_cpu_dpll2 = val;
        break;
    /* #define DPLL3_ADDRESS_88			0x18116188 */
    case 0x188:
        s->srif_unk_dpll3 = val;
        break;
    /* #define DPLL2_ADDRESS_44			0x18116244 */
    case 0x244:

        s->srif_ddr_dpll2 = val;

        break;
    /* #define CPU_DPLL3_ADDRESS			0x181161c8 */
    case 0x1c8:
        s->srif_cpu_dpll3 = val;
        break;
    /* #define DDR_DPLL3_ADDRESS			0x18116248 */
    case 0x248:

        s->srif_ddr_dpll3 = val;

        break;
    /* #define PCIe_DPLL4_ADDRESS			0x18116c0c */
    case 0xc0c:
        s->srif_pcie_dpll4 = val;
    break;

    default:

        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08lx\n", addr, val);

        break;
    }
}

static uint64_t ar71xx_ddr_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t val = 0;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {
    /* #define AR7240_DDR_CONFIG               AR7240_DDR_CTL_BASE+0 */
    case 0x00:
        val = s->ddr_cfg;
        break;
    /* #define AR7240_DDR_CONFIG2              AR7240_DDR_CTL_BASE+4 */
    case 0x04:
        val = s->ddr_cfg2;
        break; 
    /* #define AR7240_DDR_MODE                 AR7240_DDR_CTL_BASE+0x08 */
    case 0x08:
        val = s->ddr_mode;
    /* #define AR7240_DDR_EXT_MODE             AR7240_DDR_CTL_BASE+0x0c */
    case 0x0c:
        val = s->ddr_extmode;
        break;
    /* #define AR7240_DDR_CONTROL              AR7240_DDR_CTL_BASE+0x10 */
    case 0x10:
        val = s->ddr_ctrl;
        break;
    /* #define AR7240_DDR_REFRESH              AR7240_DDR_CTL_BASE+0x14 */
    case 0x14:
        val = s->ddr_refresh;
        break;
    /* #define QCA_DDR_RD_DATA_THIS_CYCLE_REG	QCA_DDR_CTRL_BASE_REG + 0x018 */
    case 0x18:
        val = s->ddr_rd_data;
        break;
    /* #define AR7240_DDR_TAP_CONTROL0         AR7240_DDR_CTL_BASE+0x1c */
    case 0x1c:
        val = s->ddr_tap_ctrl0;
        break;
    /* #define QCA_DDR_TAP_CTRL_1_REG		QCA_DDR_CTRL_BASE_REG + 0x020 */
    case 0x20:
        val = s->ddr_tap_ctrl1;
        break;
    /* #define QCA_DDR_TAP_CTRL_2_REG		QCA_DDR_CTRL_BASE_REG + 0x024 */
    case 0x24:
        val = s->ddr_tap_ctrl2;
        break;
    /* #define QCA_DDR_TAP_CTRL_3_REG		QCA_DDR_CTRL_BASE_REG + 0x028 */
    case 0x28:
        val = s->ddr_tap_ctrl3;
        break;
    /* #define QCA_DDR_PERF_COMP_ADDR_0_REG		QCA_DDR_CTRL_BASE_REG + 0x02c */
    case 0x2c:
        val = s->ddr_perf_comp_addr_0;
        break;
    /*	#define QCA_DDR_PERF_COMP_ADDR_1_REG		QCA_DDR_CTRL_BASE_REG + 0x068 */
    case 0x68:
        val = s->ddr_perf_comp_addr_1;
        break;
    /*	#define QCA_DDR_PERF_COMP_AHB_GE0_1_REG		QCA_DDR_CTRL_BASE_REG + 0x070 */
    case 0x70:
        val = s->ddr_perf_comp_ge0_1;
        break;
    /*	#define QCA_DDR_PERF_MASK_AHB_GE1_1_REG		QCA_DDR_CTRL_BASE_REG + 0x074 */
    case 0x74:
        val = s->ddr_perf_mask_ge1_1;
        break;
    /*	#define QCA_DDR_PERF_COMP_AHB_GE1_1_REG		QCA_DDR_CTRL_BASE_REG + 0x078 */
    case 0x78:
        val = s->ddr_perf_comp_ge1_1;
        break;
    /* 	#define QCA_DDR_WB_FLUSH_USB_REG		QCA_DDR_CTRL_BASE_REG + 0x0A4 */
    case 0xa4:
        val = s->ddr_wb_flush_usb >> 1;  //hack to emulate flush Write buffer
        break;
    /* #define AR7240_DDR_BURST                AR7240_DDR_CTL_BASE+0xc4 */
    case 0xc4:
        val = s->ddr_burst1; 
        break;

    /* #define AR7240_DDR_BURST2                AR7240_DDR_CTL_BASE+0xc8 */
    case 0xc8:
        val = s->ddr_burst1; 
        break;
    /* #define QCA_AHB_MASTER_TOUT_MAX_REG		QCA_DDR_CTRL_BASE_REG + 0x0CC */
    case 0xcc:
        val = s->ddr_tout_maxreg; 
        break;

    /* #define QCA_DDR_CTRL_CFG_REG			QCA_DDR_CTRL_BASE_REG + 0x108 */
    case 0x108:
       val = s->ddr_ctrl_cfg;
       break;
    /*	#define QCA_DDR_BIST_REG			QCA_DDR_CTRL_BASE_REG + 0x11C */
    case 0x11c:
        val = s->ddr_bist;
        break;
    /*	#define QCA_DDR_BIST_STATUS_REG			QCA_DDR_CTRL_BASE_REG + 0x120 */
    case 0x120:

        val = s->ddr_bist_status;
        break;


#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08x\n", addr, val);
#endif
        break;
    }
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08x\n", (uint64_t) addr &0xffffffff, val);
    return val;
}

static void ar71xx_ddr_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t saddr;

    saddr = (addr & 0xfffff);
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08lx\n", (uint64_t) addr &0xffffffff, val);
    switch (saddr) {
    /* #define AR7240_DDR_CONFIG               AR7240_DDR_CTL_BASE+0 */
    case 0x00:
        s->ddr_cfg = val;
        break;
    /* #define AR7240_DDR_CONFIG2              AR7240_DDR_CTL_BASE+4 */
    case 0x04:
        s->ddr_cfg2 = val;
        break; 
    /* #define AR7240_DDR_MODE                 AR7240_DDR_CTL_BASE+0x08 */
    case 0x08:
        s->ddr_mode = val;
        break;
    /* #define AR7240_DDR_EXT_MODE             AR7240_DDR_CTL_BASE+0x0c */
    case 0x0c:
        s->ddr_extmode = val;
        break; 
    /* #define AR7240_DDR_CONTROL              AR7240_DDR_CTL_BASE+0x10 */
    case 0x10:
        s->ddr_ctrl = val;
        break;
    /* #define AR7240_DDR_REFRESH              AR7240_DDR_CTL_BASE+0x14 */
    case 0x14:
        val = s->ddr_refresh;
        break;
    /* #define QCA_DDR_RD_DATA_THIS_CYCLE_REG	QCA_DDR_CTRL_BASE_REG + 0x018 */
    case 0x18:
        s->ddr_rd_data = val;
        break;
    /* #define AR934X_RESET_REG_RESET_MODULE		0x1c */ ////dup index with QCA_DDR_TAP_CTRL_0_REG 0x1c
    case 0x1c:
        s->ddr_tap_ctrl0 = val;
        break;
    /* #define QCA_DDR_TAP_CTRL_1_REG		QCA_DDR_CTRL_BASE_REG + 0x020 */
    case 0x20:
        s->ddr_tap_ctrl1 = val;
        break;
    /* #define QCA_DDR_TAP_CTRL_2_REG		QCA_DDR_CTRL_BASE_REG + 0x024 */
    case 0x24:
        s->ddr_tap_ctrl2 = val;
        break;
    /* #define QCA_DDR_TAP_CTRL_3_REG		QCA_DDR_CTRL_BASE_REG + 0x028 */
    case 0x28:
        s->ddr_tap_ctrl3 = val;
        break;
    /* #define QCA_DDR_PERF_COMP_ADDR_0_REG		QCA_DDR_CTRL_BASE_REG + 0x02c */
    case 0x2c:
        s->ddr_perf_comp_addr_0 = val;
        break;
    /*	#define QCA_DDR_PERF_COMP_ADDR_1_REG		QCA_DDR_CTRL_BASE_REG + 0x068 */
    case 0x68:
        s->ddr_perf_comp_addr_1 = val;
        break;
    /*	#define QCA_DDR_PERF_COMP_AHB_GE0_1_REG		QCA_DDR_CTRL_BASE_REG + 0x070 */
    case 0x70:
        s->ddr_perf_comp_ge0_1 = val;
        break;
    /*	#define QCA_DDR_PERF_MASK_AHB_GE1_1_REG		QCA_DDR_CTRL_BASE_REG + 0x074 */
    case 0x74:
        s->ddr_perf_mask_ge1_1 = val;
        break;
    /*	#define QCA_DDR_PERF_COMP_AHB_GE1_1_REG		QCA_DDR_CTRL_BASE_REG + 0x078 */
    case 0x78:
        s->ddr_perf_comp_ge1_1 = val;
        break;
    /* 	#define QCA_DDR_WB_FLUSH_USB_REG		QCA_DDR_CTRL_BASE_REG + 0x0A4 */
    case 0xa4:
        s->ddr_wb_flush_usb = val;
        break;
    /* #define AR7240_DDR_BURST                AR7240_DDR_CTL_BASE+0xc4 */
    case 0xc4:
        s->ddr_burst1 = val; 
        break;

    /* #define AR7240_DDR_BURST2                AR7240_DDR_CTL_BASE+0xc8 */
    case 0xc8:
        s->ddr_burst1 = val; 
        break;
    /* #define QCA_AHB_MASTER_TOUT_MAX_REG		QCA_DDR_CTRL_BASE_REG + 0x0CC */
    case 0xcc:
        s->ddr_tout_maxreg = val; 
        break;

    /* #define QCA_DDR_CTRL_CFG_REG			QCA_DDR_CTRL_BASE_REG + 0x108 */
    case 0x108:
       s->ddr_ctrl_cfg = val;
       break;
    /*	#define QCA_DDR_BIST_REG			QCA_DDR_CTRL_BASE_REG + 0x11C */
    case 0x11c:
        s->ddr_bist = val;
        if (s->ddr_bist==0x1) {
          s->ddr_bist_status = 0x11;
        } else {
          s->ddr_bist_status = 0x0;
        }
        break;
    /*	#define QCA_DDR_BIST_STATUS_REG			QCA_DDR_CTRL_BASE_REG + 0x120 */
    case 0x120:
        s->ddr_bist_status = val;
        break;

    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08lx\n", addr, val);
#endif
        break;
    }
}

static uint64_t ar71xx_spi_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t val = 0;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {
    /* #define QCA_SPI_FUNC_SEL_REG		QCA_FLASH_BASE_REG + 0x00 */
    case 0x00:
        val = s->spi_func_sel;
        break;
    /* #define QCA_SPI_CTRL_REG		QCA_FLASH_BASE_REG + 0x04 */
    case 0x04:
        val = s->spi_ctrl;
        break; 
    /* #define QCA_SPI_IO_CTRL_REG		QCA_FLASH_BASE_REG + 0x08 */
    case 0x08:
        val = s->spi_io_ctrl;
        break;
    /* #define QCA_SPI_READ_DATA_REG		QCA_FLASH_BASE_REG + 0x0C */
    case 0x0c:
        val = s->spi_read_data;
        break; 
    /* #define QCA_SPI_SHIFT_DATAOUT_REG	QCA_FLASH_BASE_REG + 0x10 */
    case 0x10:
        val = s->spi_shift_dataout;
        break;
    /* #define QCA_SPI_SHIFT_CNT_REG		QCA_FLASH_BASE_REG + 0x14 */
    case 0x14:
        val =  s->spi_shift_cnt;
        break;
    /* #define QCA_SPI_SHIFT_DATAIN_REG	QCA_FLASH_BASE_REG + 0x18 */
    case 0x18:
        val = s->spi_shift_datain;
        break;


    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08x\n", addr, val);
#endif
        break;
    }
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08x\n", (uint64_t) addr &0xffffffff, val);
    return val;
}

static void ar71xx_spi_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {
    /* #define QCA_SPI_FUNC_SEL_REG		QCA_FLASH_BASE_REG + 0x00 */
    case 0x00:
        s->spi_func_sel = val;
        break;
    /* #define QCA_SPI_CTRL_REG		QCA_FLASH_BASE_REG + 0x04 */
    case 0x04:
        s->spi_ctrl = val;
        break; 
    /* #define QCA_SPI_IO_CTRL_REG		QCA_FLASH_BASE_REG + 0x08 */
    case 0x08:
        s->spi_io_ctrl = val;


        break;
    /* #define QCA_SPI_READ_DATA_REG		QCA_FLASH_BASE_REG + 0x0C */
    case 0x0c:
        s->spi_read_data = val;
        break; 
    /* #define QCA_SPI_SHIFT_DATAOUT_REG	QCA_FLASH_BASE_REG + 0x10 */
    case 0x10:
        s->spi_shift_dataout = val;
        break;
    /* #define QCA_SPI_SHIFT_CNT_REG		QCA_FLASH_BASE_REG + 0x14 */
    case 0x14:
        s->spi_shift_cnt = val;
        break;
    /* #define QCA_SPI_SHIFT_DATAIN_REG	QCA_FLASH_BASE_REG + 0x18 */
    case 0x18:
        s->spi_shift_datain = val;
        break;


    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08lx\n", addr, val);
#endif
        break;
    }
}

static uint64_t ar71xx_gpio_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t val = 0;
    uint32_t saddr;

    saddr = (addr & 0xfffff);

    switch (saddr) {

    /* #define QCA_GPIO_OE_REG			QCA_GPIO_BASE_REG + 0x00 */
    case 0x00:
       val = s->gpio_oe;

        break;

    /* #define QCA_GPIO_IN_REG			QCA_GPIO_BASE_REG + 0x04 */
    case 0x04:
        val = s->gpio_in;

        break; 

    /* #define QCA_GPIO_OUT_REG		QCA_GPIO_BASE_REG + 0x08 */
    case 0x08:

        val = s->gpio_out;
    /* #define AR7240_GPIO_FUNC                AR7240_GPIO_BASE+0x28 */
    case 0x28:
        val = s->gpio_out_func;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC1 0x30 */
    case 0x30:
        val = s->gpio_out_func1;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC2 0x34 */
    case 0x34:
        val = s->gpio_out_func2;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC3 0x38 */
    case 0x38:
        val = s->gpio_out_func3;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC4 0x3c */
    case 0x3c:
        val = s->gpio_out_func4;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC5 0x40 */
    case 0x40:
        val = s->gpio_out_func5;

        break;
    /* #define AR934X_GPIO_IN_ENABLE0         AR934X_GPIO_BASE + 0x44 */
    case 0x44:
        val = s->gpio_in_en0;

        break;
    /* #define AR934X_GPIO_IN_ENABLE1         AR934X_GPIO_BASE + 0x48 */
    case 0x48:

        val = s->gpio_in_en1;

        break;
    /* #define AR934X_GPIO_IN_ENABLE9         AR934X_GPIO_BASE + 0x68 */
    case 0x68:

        val = s->gpio_in_en9;

        break;
    /* #define AR934X_GPIO_REG_FUNC 0x6c */
    case 0x6c:
        val = s->gpio_func;
        break;

    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08x\n", addr, val);
#endif
        break;
    }
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08x\n", (uint64_t) addr &0xffffffff, val);
    return val;
}

static void ar71xx_gpio_write(void *opaque, hwaddr addr,
                             uint64_t val, unsigned size)
{
    AR71xxFPGAState *s = opaque;
    uint32_t saddr;

    saddr = (addr & 0xfffff);
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08lx\n", (uint64_t) addr &0xffffffff, val);
    switch (saddr) {
    /* #define QCA_GPIO_OE_REG			QCA_GPIO_BASE_REG + 0x00 */
    case 0x00:
        s->gpio_oe = val;
        break;

    /* #define QCA_GPIO_IN_REG			QCA_GPIO_BASE_REG + 0x04 */
    case 0x04:
        s->gpio_in = val;
        break; 

    /* #define QCA_GPIO_OUT_REG		QCA_GPIO_BASE_REG + 0x08 */
    case 0x08:
        s->gpio_out = val;

        break;
    /* #define AR7240_GPIO_FUNC                AR7240_GPIO_BASE+0x28 */
    case 0x28:
        s->gpio_out_func = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC0 0x2c */
    case 0x2c:
        s->gpio_out_func0 = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC1 0x30 */
    case 0x30:
        s->gpio_out_func1 = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC2 0x34 */
    case 0x34:
        s->gpio_out_func2 = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC3 0x38 */
    case 0x38:
        s->gpio_out_func3 = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC4 0x3c */
    case 0x3c:
        s->gpio_out_func4 = val;
        break;
    /* #define AR934X_GPIO_REG_OUT_FUNC5 0x40 */
    case 0x40:
        s->gpio_out_func5 = val;

        break;
    /* #define AR934X_GPIO_IN_ENABLE0         AR934X_GPIO_BASE + 0x44 */
    case 0x44:

        s->gpio_in_en0 = val;

        break;
    /* #define AR934X_GPIO_IN_ENABLE1         AR934X_GPIO_BASE + 0x48 */
    case 0x48:

        s->gpio_in_en1 = val;

        break;
    /* #define AR934X_GPIO_IN_ENABLE9         AR934X_GPIO_BASE + 0x68 */
    case 0x68:

        s->gpio_in_en9 = val;

        break;
    /* #define AR934X_GPIO_REG_FUNC 0x6c */
    case 0x6c:
        s->gpio_func = val;
        break;

    default:
#if 1
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08lx\n", addr, val);
#endif
        break;
    }
}


static const MemoryRegionOps ar71xx_fpga_ops = {
    .read = ar71xx_fpga_read,
    .write = ar71xx_fpga_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ar71xx_spi_ops = {
    .read = ar71xx_spi_read,
    .write = ar71xx_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ar71xx_gpio_ops = {
    .read = ar71xx_gpio_read,
    .write = ar71xx_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const MemoryRegionOps ar71xx_ddr_ops = {
    .read = ar71xx_ddr_read,
    .write = ar71xx_ddr_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ar71xx_fpga_reset(void *opaque)
{
    AR71xxFPGAState *s = opaque;


s->pll_cpu = 0x40021380;

    s->wdog_ctrl = 0x00;
s->ddr_wmac_7044 = 0x3;
s->cpu_pll_dither = 0xc00000;
s->srif_cpu_dpll2 = 0xD0800F00;
s->srif_pll_dpll2 = 0x50802F00;
s->srif_cpu_dpll4 = 0x0729f808;  //hack for now to get mask CPU_DPLL4_MEAS_DONE_MASK		0x00000008 to work
s->srif_ddr_dpll4 = 0xec6dff18;  //hack for now to get mask DDR_DPLL4_MEAS_DONE_MASK		0x00000008 to work
        s->srif_pcie_dpll4 = 0xff8e05f8;
s->reset_reg_reset = 0x24000008;
s->gpio_in = 0x73FFF9;
s->gpio_func = 0x42;
s->gpio_out = 0x720021;
s->gpio_oe = 0x20FBD0;
s->gpio_in_en0 = 0x908;
s->gpio_out_func2 = 0x3180000;
s->gpio_out_func3 = 0x0;
s->gpio_out_func4 = 0x2E2F004F;
s->gpio_out_func5 = 0x0;
       s->ddr_bist_status = 0x0;

s->ddr_tout_maxreg = 0xffff;
s->spi_shift_datain = 0x010216;


}

static AR71xxFPGAState *ar71xx_fpga_init(MemoryRegion *address_space,
         hwaddr base, qemu_irq uart_irq, Chardev *uart_chr)
{
    AR71xxFPGAState *s;

    s = (AR71xxFPGAState *)g_malloc0(sizeof(AR71xxFPGAState));
/*
CPU Mapped Registers Summary
Address Description Page
0x18000000–0x18000128 DDR Registers 
0x18020000–0x18020018 UART0 (Low-Speed) Registers 
0x18040000–0x1804006C GPIO Registers 
0x18050000–0x18050048 PLL Control Registers 
0x18060000–0x1806405C Reset Control Registers 
0x18070000–0x18070010 GMAC Interface Registers 
0x18080000–0x1808305C GMAC0 Ingress NAT/Egress NAT Registers 
0x180A0000–0x180A006C MBOX Registers 
0x180A9000–0x180A9024 SLIC Registers 
0x180B0000–0x180B0018 Stereo Registers 
0x180B8000–0x180B8024 MDIO Registers 
         --PCI RC--
0x180c0000-0x180c1000 AR724X_PCI_CRP Registers
0x180f0000-0x180f0100 AR724X_PCI_CTRL Registers
         --      --
0x180C0000–0x180F005C PCIE RC Registers 
0x18100008–0x18100104 WDMA Registers 
0x18100800–0x18100A44 WQCU Registers 
0x18101000–0x18101F04 WDCU Registers
0x18107000–0x18107058 RTC Registers 
0x18108000–0x1810E000 WPCU Registers 
0x180C0000–0x180C003E PCIE Configuration Space Registers 
0x18400000–0x18400054 Checksum Registers
0x18500000–0x18500010 UART1 (High-Speed) Registers 
0x19000000–0x190001D8 GMAC0/GMAC1 Registers 
0x1A000000–0x1A0001D8

0x1B000100–0x1B00017C USB Controller Registers 
0x1B000200–0x1B0002B4 NAND Flash Registers
0x18127800–0x18127D18, PCIE EP DMA Registers 
0x00000000–0x00000F18

0x1F000000–0x1F000018 Serial Flash SPI Registers 
0x0000–0x00FC Ethernet Switch Registers 
0x0100–0x0124
 
*/
/* Hack unknown register...not documented */
uint8_t x[2] = {0x1f, 0x20};
cpu_physical_memory_write(0x1d007f00, x, 1);
uint8_t y[2] = {0x0, 0x1f};
cpu_physical_memory_write(0x1d007f10, y, 1);
/*  ** ** ** ** ** ** ** ** ** ** ** **   */

    memory_region_init_io(&s->iomem, NULL, &ar71xx_fpga_ops, s,
                          "ar71xx-fpga", 0x1fffff);

    memory_region_init_io(&s->iomem_gpio, NULL, &ar71xx_gpio_ops, s,
                          "ar71xx_gpio", 0x6c+1);
    memory_region_init_io(&s->iomem_ddr, NULL, &ar71xx_ddr_ops, s,
                          "ar71xx-ddr", 0x128+1);
    memory_region_init_io(&s->iomem_spi, NULL, &ar71xx_spi_ops, s,
                          "ar71xx-spi", 0x18+1);

    memory_region_init_alias(&s->iomem_usb, NULL, "ar71xx_usb_ctrl_base",
                             &s->iomem, 0, 0x7c+1);
    memory_region_init_alias(&s->iomem_reset, NULL, "ar71xx_reset_base",
                             &s->iomem, 0, 0x405c+1);
    memory_region_init_alias(&s->iomem_pll, NULL, "ar71xx_pll_base",
                             &s->iomem, 0, 0x48+1);
    memory_region_init_alias(&s->iomem_gmac, NULL, "ar934x_gmac_base",
                             &s->iomem, 0, 0x10+1);
    memory_region_init_alias(&s->iomem_pci_crp, NULL, "ar934x_pci_crp_base",
                             &s->iomem, 0, 0x1000+1);
    memory_region_init_alias(&s->iomem_pci_ctrl, NULL, "ar934x_pci_ctrl_base",
                             &s->iomem, 0, 0x100+1);
    memory_region_init_alias(&s->iomem_wdma, NULL, "ar934x_wdma_base",
                             &s->iomem, 0, 0x104+1);
    memory_region_init_alias(&s->iomem_wqcu, NULL, "ar934x_wqcu_base",
                             &s->iomem, 0, 0xa44+1);
    memory_region_init_alias(&s->iomem_wdcu, NULL, "ar934x_wdcu_base",
                             &s->iomem, 0, 0xf04+1);
    memory_region_init_alias(&s->iomem_rtc, NULL, "ar934x_rtc_base",
                             &s->iomem, 0, 0x58+1);
    memory_region_init_alias(&s->iomem_wpcu, NULL, "ar934x_wpcu_base",
                             &s->iomem, 0, 0xe000+1);
    memory_region_init_alias(&s->iomem_srif, NULL, "ar934x_srif_base",
                             &s->iomem, 0, 0x1000+1);
    memory_region_init_alias(&s->iomem_unk, NULL, "ar934x_unk_base",
                             &s->iomem, 0, 0x2000+1);
    memory_region_init_alias(&s->iomem_ge0, NULL, "ar71xx_ge0_base",
                             &s->iomem, 0, 0x1d8+1);
    memory_region_init_alias(&s->iomem_ge1, NULL, "ar71xx_ge1_base",
                             &s->iomem, 0, 0x1d8+1);
    memory_region_init_alias(&s->iomem_ehci, NULL, "ar934x_ehci_base",
                             &s->iomem, 0, 0x200+1);


    memory_region_init_alias(&s->iomem_ohci, NULL, "ar71xx_ohci_base",
                             &s->iomem, 0, 0x1000+1);


    memory_region_add_subregion(address_space, base + 0x00000000, &s->iomem_ddr);

    memory_region_add_subregion(address_space, base + 0x00040000, &s->iomem_gpio);
    memory_region_add_subregion(address_space, base + 0x00050000, &s->iomem_pll);
    memory_region_add_subregion(address_space, base + 0x00060000, &s->iomem_reset);
    memory_region_add_subregion(address_space, base + 0x000c0000, &s->iomem_pci_crp);
    memory_region_add_subregion(address_space, base + 0x000f0000, &s->iomem_pci_ctrl);
    memory_region_add_subregion(address_space, base + 0x00070000, &s->iomem_gmac);
    memory_region_add_subregion(address_space, base + 0x00100008, &s->iomem_wdma);
    memory_region_add_subregion(address_space, base + 0x00100800, &s->iomem_wqcu);
    memory_region_add_subregion(address_space, base + 0x00101000, &s->iomem_wdcu);
    memory_region_add_subregion(address_space, base + 0x00107000, &s->iomem_rtc);
    memory_region_add_subregion(address_space, base + 0x00108000, &s->iomem_wpcu);

    memory_region_add_subregion(address_space, base + 0x00116000, &s->iomem_srif);
    memory_region_add_subregion(address_space, base + 0x00130000, &s->iomem_unk);
    memory_region_add_subregion(address_space, 0x19000000, &s->iomem_ge0);
    memory_region_add_subregion(address_space, 0x1a000000, &s->iomem_ge1);
    memory_region_add_subregion(address_space, 0x1b000000, &s->iomem_ehci);
    memory_region_add_subregion(address_space, 0x1b000100, &s->iomem_usb);

    memory_region_add_subregion(address_space, 0x1c000000, &s->iomem_ohci);
    memory_region_add_subregion(address_space, 0x1f000000, &s->iomem_spi);



 //   s->uart = serial_mm_init(address_space, base + 0x900, 3, uart_irq,
   //                          230400, uart_chr, DEVICE_NATIVE_ENDIAN);
#if 1
s->uart_ls = serial_mm_init(address_space, base + 0x20000, 2, uart_irq,
                             115200, uart_chr, DEVICE_NATIVE_ENDIAN);
#else
//s->uart_hs = serial_mm_init(address_space, base + 0x500000, 3, uart_irq,
  //                           230400, uart_chr, DEVICE_NATIVE_ENDIAN);
    /* Allocate uart */
    if (serial_hd(1)) {
        ath_apbuart_create(base + 0x500000, serial_hd(1), uart_irq);
    }
#endif
    ar71xx_fpga_reset(s);
    qemu_register_reset(ar71xx_fpga_reset, s);

    return s;
}

/* Network support */
static void network_init(PCIBus *pci_bus)
{
    int i;

    for(i = 0; i < nb_nics; i++) {
        NICInfo *nd = &nd_table[i];
        const char *default_devaddr = NULL;

        if (i == 0 && (!nd->model || strcmp(nd->model, "pcnet") == 0))
            /* The ar71xx board has a PCNet card using PCI SLOT 11 */
            default_devaddr = "0b";

        pci_nic_init_nofail(nd, pci_bus, "pcnet", default_devaddr);
    }
}

static void write_bootloader_nanomips(uint8_t *base, int64_t run_addr,
                                      int64_t kernel_entry)
{
    uint16_t *p;

    /* Small bootloader */
    p = (uint16_t *)base;
DB_PRINT("*************bootloader_NANOMIPS 0x%hn\n",p);
#define NM_HI1(VAL) (((VAL) >> 16) & 0x1f)
#define NM_HI2(VAL) \
          (((VAL) & 0xf000) | (((VAL) >> 19) & 0xffc) | (((VAL) >> 31) & 0x1))
#define NM_LO(VAL)  ((VAL) & 0xfff)

    stw_p(p++, 0x2800); stw_p(p++, 0x001c);
                                /* bc to_here */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */
    stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop */

    /* to_here: */
    if (semihosting_get_argc()) {
        /* Preserve a0 content as arguments have been passed    */
        stw_p(p++, 0x8000); stw_p(p++, 0xc000);
                                /* nop                          */
    } else {
        stw_p(p++, 0x0080); stw_p(p++, 0x0002);
                                /* li a0,2                      */
    }

    stw_p(p++, 0xe3a0 | NM_HI1(ENVP_ADDR - 64));

    stw_p(p++, NM_HI2(ENVP_ADDR - 64));
                                /* lui sp,%hi(ENVP_ADDR - 64)   */

    stw_p(p++, 0x83bd); stw_p(p++, NM_LO(ENVP_ADDR - 64));
                                /* ori sp,sp,%lo(ENVP_ADDR - 64) */

    stw_p(p++, 0xe0a0 | NM_HI1(ENVP_ADDR));

    stw_p(p++, NM_HI2(ENVP_ADDR));
                                /* lui a1,%hi(ENVP_ADDR)        */

    stw_p(p++, 0x80a5); stw_p(p++, NM_LO(ENVP_ADDR));
                                /* ori a1,a1,%lo(ENVP_ADDR)     */

    stw_p(p++, 0xe0c0 | NM_HI1(ENVP_ADDR + 8));

    stw_p(p++, NM_HI2(ENVP_ADDR + 8));
                                /* lui a2,%hi(ENVP_ADDR + 8)    */

    stw_p(p++, 0x80c6); stw_p(p++, NM_LO(ENVP_ADDR + 8));
                                /* ori a2,a2,%lo(ENVP_ADDR + 8) */

    stw_p(p++, 0xe0e0 | NM_HI1(loaderparams.ram_low_size));

    stw_p(p++, NM_HI2(loaderparams.ram_low_size));
                                /* lui a3,%hi(loaderparams.ram_low_size) */

    stw_p(p++, 0x80e7); stw_p(p++, NM_LO(loaderparams.ram_low_size));
                                /* ori a3,a3,%lo(loaderparams.ram_low_size) */

    /*
     * Load BAR registers as done by YAMON:
     *
     *  - set up PCI0 I/O BARs from 0x18000000 to 0x181fffff
     *  - set up PCI0 MEM0 at 0x10000000, size 0x8000000
     *  - set up PCI0 MEM1 at 0x18200000, size 0xbe00000
     *
     */
    stw_p(p++, 0xe040); stw_p(p++, 0x0681);
                                /* lui t1, %hi(0xb4000000)      */

#ifdef TARGET_WORDS_BIGENDIAN

    stw_p(p++, 0xe020); stw_p(p++, 0x0be1);
                                /* lui t0, %hi(0xdf000000)      */

    /* 0x68 corresponds to GT_ISD (from hw/mips/gt64xxx_pci.c)  */
    stw_p(p++, 0x8422); stw_p(p++, 0x9068);
                                /* sw t0, 0x68(t1)              */

    stw_p(p++, 0xe040); stw_p(p++, 0x077d);
                                /* lui t1, %hi(0xbbe00000)      */

    stw_p(p++, 0xe020); stw_p(p++, 0x0801);
                                /* lui t0, %hi(0xc0000000)      */

    /* 0x48 corresponds to GT_PCI0IOLD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9048);
                                /* sw t0, 0x48(t1)              */

    stw_p(p++, 0xe020); stw_p(p++, 0x0800);
                                /* lui t0, %hi(0x40000000)      */

    /* 0x50 corresponds to GT_PCI0IOHD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9050);
                                /* sw t0, 0x50(t1)              */

    stw_p(p++, 0xe020); stw_p(p++, 0x0001);
                                /* lui t0, %hi(0x80000000)      */

    /* 0x58 corresponds to GT_PCI0M0LD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9058);
                                /* sw t0, 0x58(t1)              */

    stw_p(p++, 0xe020); stw_p(p++, 0x07e0);
                                /* lui t0, %hi(0x3f000000)      */

    /* 0x60 corresponds to GT_PCI0M0HD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9060);
                                /* sw t0, 0x60(t1)              */

    stw_p(p++, 0xe020); stw_p(p++, 0x0821);
                                /* lui t0, %hi(0xc1000000)      */

    /* 0x80 corresponds to GT_PCI0M1LD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9080);
                                /* sw t0, 0x80(t1)              */

    stw_p(p++, 0xe020); stw_p(p++, 0x0bc0);
                                /* lui t0, %hi(0x5e000000)      */

#else

    stw_p(p++, 0x0020); stw_p(p++, 0x00df);
                                /* addiu[32] t0, $0, 0xdf       */

    /* 0x68 corresponds to GT_ISD                               */
    stw_p(p++, 0x8422); stw_p(p++, 0x9068);
                                /* sw t0, 0x68(t1)              */

    /* Use kseg2 remapped address 0x1be00000                    */
    stw_p(p++, 0xe040); stw_p(p++, 0x077d);
                                /* lui t1, %hi(0xbbe00000)      */

    stw_p(p++, 0x0020); stw_p(p++, 0x00c0);
                                /* addiu[32] t0, $0, 0xc0       */

    /* 0x48 corresponds to GT_PCI0IOLD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9048);
                                /* sw t0, 0x48(t1)              */

    stw_p(p++, 0x0020); stw_p(p++, 0x0040);
                                /* addiu[32] t0, $0, 0x40       */

    /* 0x50 corresponds to GT_PCI0IOHD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9050);
                                /* sw t0, 0x50(t1)              */

    stw_p(p++, 0x0020); stw_p(p++, 0x0080);
                                /* addiu[32] t0, $0, 0x80       */

    /* 0x58 corresponds to GT_PCI0M0LD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9058);
                                /* sw t0, 0x58(t1)              */

    stw_p(p++, 0x0020); stw_p(p++, 0x003f);
                                /* addiu[32] t0, $0, 0x3f       */

    /* 0x60 corresponds to GT_PCI0M0HD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9060);
                                /* sw t0, 0x60(t1)              */

    stw_p(p++, 0x0020); stw_p(p++, 0x00c1);
                                /* addiu[32] t0, $0, 0xc1       */

    /* 0x80 corresponds to GT_PCI0M1LD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9080);
                                /* sw t0, 0x80(t1)              */

    stw_p(p++, 0x0020); stw_p(p++, 0x005e);
                                /* addiu[32] t0, $0, 0x5e       */

#endif

    /* 0x88 corresponds to GT_PCI0M1HD                          */
    stw_p(p++, 0x8422); stw_p(p++, 0x9088);
                                /* sw t0, 0x88(t1)              */

    stw_p(p++, 0xe320 | NM_HI1(kernel_entry));

    stw_p(p++, NM_HI2(kernel_entry));
                                /* lui t9,%hi(kernel_entry)     */

    stw_p(p++, 0x8339); stw_p(p++, NM_LO(kernel_entry));
                                /* ori t9,t9,%lo(kernel_entry)  */

    stw_p(p++, 0x4bf9); stw_p(p++, 0x0000);
                                /* jalrc   t8                   */
}

/* ROM and pseudo bootloader

   The following code implements a very very simple bootloader. It first
   loads the registers a0 to a3 to the values expected by the OS, and
   then jump at the kernel address.

   The bootloader should pass the locations of the kernel arguments and
   environment variables tables. Those tables contain the 32-bit address
   of NULL terminated strings. The environment variables table should be
   terminated by a NULL address.

   For a simpler implementation, the number of kernel arguments is fixed
   to two (the name of the kernel and the command line), and the two
   tables are actually the same one.

   The registers a0 to a3 should contain the following values:
     a0 - number of kernel arguments
     a1 - 32-bit address of the kernel arguments table
     a2 - 32-bit address of the environment variables table
     a3 - RAM size in bytes
*/
static void write_bootloader(uint8_t *base, int64_t run_addr,
                             int64_t kernel_entry)
{
    uint32_t *p;

    /* Small bootloader */
    p = (uint32_t *)base;
DB_PRINT("*************bootloader 0x%p\n",p);
    stl_p(p++, 0x08000000 |                                      /* j 0x1fc00580 */
                 ((run_addr + 0x580) & 0x0fffffff) >> 2);
    stl_p(p++, 0x00000000);                                      /* nop */

    /* YAMON service vector */
    stl_p(base + 0x500, run_addr + 0x0580);      /* start: */
    stl_p(base + 0x504, run_addr + 0x083c);      /* print_count: */
    stl_p(base + 0x520, run_addr + 0x0580);      /* start: */
    stl_p(base + 0x52c, run_addr + 0x0800);      /* flush_cache: */
    stl_p(base + 0x534, run_addr + 0x0808);      /* print: */
    stl_p(base + 0x538, run_addr + 0x0800);      /* reg_cpu_isr: */
    stl_p(base + 0x53c, run_addr + 0x0800);      /* unred_cpu_isr: */
    stl_p(base + 0x540, run_addr + 0x0800);      /* reg_ic_isr: */
    stl_p(base + 0x544, run_addr + 0x0800);      /* unred_ic_isr: */
    stl_p(base + 0x548, run_addr + 0x0800);      /* reg_esr: */
    stl_p(base + 0x54c, run_addr + 0x0800);      /* unreg_esr: */
    stl_p(base + 0x550, run_addr + 0x0800);      /* getchar: */
    stl_p(base + 0x554, run_addr + 0x0800);      /* syscon_read: */


    /* Second part of the bootloader */
    p = (uint32_t *) (base + 0x580);

    if (semihosting_get_argc()) {
        /* Preserve a0 content as arguments have been passed */
        stl_p(p++, 0x00000000);                         /* nop */
    } else {
        stl_p(p++, 0x24040002);                         /* addiu a0, zero, 2 */
    }
    stl_p(p++, 0x3c1d0000 | (((ENVP_ADDR - 64) >> 16) & 0xffff)); /* lui sp, high(ENVP_ADDR) */
    stl_p(p++, 0x37bd0000 | ((ENVP_ADDR - 64) & 0xffff));        /* ori sp, sp, low(ENVP_ADDR) */
    stl_p(p++, 0x3c050000 | ((ENVP_ADDR >> 16) & 0xffff));       /* lui a1, high(ENVP_ADDR) */
    stl_p(p++, 0x34a50000 | (ENVP_ADDR & 0xffff));               /* ori a1, a1, low(ENVP_ADDR) */
    stl_p(p++, 0x3c060000 | (((ENVP_ADDR + 8) >> 16) & 0xffff)); /* lui a2, high(ENVP_ADDR + 8) */
    stl_p(p++, 0x34c60000 | ((ENVP_ADDR + 8) & 0xffff));         /* ori a2, a2, low(ENVP_ADDR + 8) */
    stl_p(p++, 0x3c070000 | (loaderparams.ram_low_size >> 16));     /* lui a3, high(ram_low_size) */
    stl_p(p++, 0x34e70000 | (loaderparams.ram_low_size & 0xffff));  /* ori a3, a3, low(ram_low_size) */

    /* Load BAR registers as done by YAMON */
    stl_p(p++, 0x3c09b400);                                      /* lui t1, 0xb400 */

#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c08df00);                                      /* lui t0, 0xdf00 */
#else
    stl_p(p++, 0x340800df);                                      /* ori t0, r0, 0x00df */
#endif
    stl_p(p++, 0xad280068);                                      /* sw t0, 0x0068(t1) */

    stl_p(p++, 0x3c09bbe0);                                      /* lui t1, 0xbbe0 */

#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c08c000);                                      /* lui t0, 0xc000 */
#else
    stl_p(p++, 0x340800c0);                                      /* ori t0, r0, 0x00c0 */
#endif
    stl_p(p++, 0xad280048);                                      /* sw t0, 0x0048(t1) */
#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c084000);                                      /* lui t0, 0x4000 */
#else
    stl_p(p++, 0x34080040);                                      /* ori t0, r0, 0x0040 */
#endif
    stl_p(p++, 0xad280050);                                      /* sw t0, 0x0050(t1) */

#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c088000);                                      /* lui t0, 0x8000 */
#else
    stl_p(p++, 0x34080080);                                      /* ori t0, r0, 0x0080 */
#endif
    stl_p(p++, 0xad280058);                                      /* sw t0, 0x0058(t1) */
#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c083f00);                                      /* lui t0, 0x3f00 */
#else
    stl_p(p++, 0x3408003f);                                      /* ori t0, r0, 0x003f */
#endif
    stl_p(p++, 0xad280060);                                      /* sw t0, 0x0060(t1) */

#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c08c100);                                      /* lui t0, 0xc100 */
#else
    stl_p(p++, 0x340800c1);                                      /* ori t0, r0, 0x00c1 */
#endif
    stl_p(p++, 0xad280080);                                      /* sw t0, 0x0080(t1) */
#ifdef TARGET_WORDS_BIGENDIAN
    stl_p(p++, 0x3c085e00);                                      /* lui t0, 0x5e00 */
#else
    stl_p(p++, 0x3408005e);                                      /* ori t0, r0, 0x005e */
#endif
    stl_p(p++, 0xad280088);                                      /* sw t0, 0x0088(t1) */

    /* Jump to kernel code */
    stl_p(p++, 0x3c1f0000 | ((kernel_entry >> 16) & 0xffff));    /* lui ra, high(kernel_entry) */
    stl_p(p++, 0x37ff0000 | (kernel_entry & 0xffff));            /* ori ra, ra, low(kernel_entry) */
    stl_p(p++, 0x03e00009);                                      /* jalr ra */
    stl_p(p++, 0x00000000);                                      /* nop */

    /* YAMON subroutines */
    p = (uint32_t *) (base + 0x800);
    stl_p(p++, 0x03e00009);                                     /* jalr ra */
    stl_p(p++, 0x24020000);                                     /* li v0,0 */
    /* 808 YAMON print */
    stl_p(p++, 0x03e06821);                                     /* move t5,ra */
    stl_p(p++, 0x00805821);                                     /* move t3,a0 */
    stl_p(p++, 0x00a05021);                                     /* move t2,a1 */
    stl_p(p++, 0x91440000);                                     /* lbu a0,0(t2) */
    stl_p(p++, 0x254a0001);                                     /* addiu t2,t2,1 */
    stl_p(p++, 0x10800005);                                     /* beqz a0,834 */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x0ff0021c);                                     /* jal 870 */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x1000fff9);                                     /* b 814 */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x01a00009);                                     /* jalr t5 */
    stl_p(p++, 0x01602021);                                     /* move a0,t3 */
    /* 0x83c YAMON print_count */
    stl_p(p++, 0x03e06821);                                     /* move t5,ra */
    stl_p(p++, 0x00805821);                                     /* move t3,a0 */
    stl_p(p++, 0x00a05021);                                     /* move t2,a1 */
    stl_p(p++, 0x00c06021);                                     /* move t4,a2 */
    stl_p(p++, 0x91440000);                                     /* lbu a0,0(t2) */
    stl_p(p++, 0x0ff0021c);                                     /* jal 870 */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x254a0001);                                     /* addiu t2,t2,1 */
    stl_p(p++, 0x258cffff);                                     /* addiu t4,t4,-1 */
    stl_p(p++, 0x1580fffa);                                     /* bnez t4,84c */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x01a00009);                                     /* jalr t5 */
    stl_p(p++, 0x01602021);                                     /* move a0,t3 */
    /* 0x870 */
    stl_p(p++, 0x3c08b800);                                     /* lui t0 b400 */
    stl_p(p++, 0x350803f8);                                     /* ori t0,t0 3f8 */
    stl_p(p++, 0x91090005);                                     /* lbu t1,5(t0) */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x31290040);                                     /* andi t1,t1 40 */
    stl_p(p++, 0x1120fffc);                                     /* beqz t1,878 <outch+0x8> */
    stl_p(p++, 0x00000000);                                     /* nop */
    stl_p(p++, 0x03e00009);                                     /* jalr ra */
    stl_p(p++, 0xa1040000);                                     /* sb a0,0(t0) */

}

static void GCC_FMT_ATTR(3, 4) prom_set(uint32_t* prom_buf, int index,
                                        const char *string, ...)
{
    va_list ap;
    int32_t table_addr;

    if (index >= ENVP_NB_ENTRIES)
        return;

    if (string == NULL) {
        prom_buf[index] = 0;
        return;
    }

    table_addr = sizeof(int32_t) * ENVP_NB_ENTRIES + index * ENVP_ENTRY_SIZE;
    prom_buf[index] = tswap32(ENVP_ADDR + table_addr);

    va_start(ap, string);
    vsnprintf((char *)prom_buf + table_addr, ENVP_ENTRY_SIZE, string, ap);
    va_end(ap);
}

/* Kernel */
static int64_t load_kernel (void)
{
    int64_t kernel_entry, kernel_high, initrd_size;
    long kernel_size;
    ram_addr_t initrd_offset;
    int big_endian;
    uint32_t *prom_buf;
    long prom_size;
    int prom_index = 0;
    uint64_t (*xlate_to_kseg0) (void *opaque, uint64_t addr);

#ifdef TARGET_WORDS_BIGENDIAN
    big_endian = 1;
#else
    big_endian = 0;
#endif

    kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys,
                           NULL, (uint64_t *)&kernel_entry, NULL,
                           (uint64_t *)&kernel_high, big_endian, EM_MIPS, 1, 0);
    if (kernel_size < 0) {
        error_report("could not load kernel '%s': %s",
                     loaderparams.kernel_filename,
                     load_elf_strerror(kernel_size));
        exit(1);
    }

    /* Check where the kernel has been linked */
    if (kernel_entry & 0x80000000ll) {
        if (kvm_enabled()) {
            error_report("KVM guest kernels must be linked in useg. "
                         "Did you forget to enable CONFIG_KVM_GUEST?");
            exit(1);
        }

        xlate_to_kseg0 = cpu_mips_phys_to_kseg0;
    } else {
        /* if kernel entry is in useg it is probably a KVM T&E kernel */
        mips_um_ksegs_enable();

        xlate_to_kseg0 = cpu_mips_kvm_um_phys_to_kseg0;
    }

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size (loaderparams.initrd_filename);
        if (initrd_size > 0) {
            /* The kernel allocates the bootmap memory in the low memory after
               the initrd.  It takes at most 128kiB for 2GB RAM and 4kiB
               pages.  */
            initrd_offset = (loaderparams.ram_low_size - initrd_size
                             - (128 * KiB)
                             - ~INITRD_PAGE_MASK) & INITRD_PAGE_MASK;
            if (kernel_high >= initrd_offset) {
                error_report("memory too small for initial ram disk '%s'",
                             loaderparams.initrd_filename);
                exit(1);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                                              initrd_offset,
                                              ram_size - initrd_offset);
        }
        if (initrd_size == (target_ulong) -1) {
            error_report("could not load initial ram disk '%s'",
                         loaderparams.initrd_filename);
            exit(1);
        }
    }

    /* Setup prom parameters. */
    prom_size = ENVP_NB_ENTRIES * (sizeof(int32_t) + ENVP_ENTRY_SIZE);
    prom_buf = g_malloc(prom_size);

    prom_set(prom_buf, prom_index++, "%s", loaderparams.kernel_filename);
    if (initrd_size > 0) {
        prom_set(prom_buf, prom_index++, "rd_start=0x%" PRIx64 " rd_size=%" PRId64 " %s",
                 xlate_to_kseg0(NULL, initrd_offset), initrd_size,
                 loaderparams.kernel_cmdline);
    } else {
        prom_set(prom_buf, prom_index++, "%s", loaderparams.kernel_cmdline);
    }

    prom_set(prom_buf, prom_index++, "memsize");
    prom_set(prom_buf, prom_index++, "%u", loaderparams.ram_low_size);

    prom_set(prom_buf, prom_index++, "ememsize");
    prom_set(prom_buf, prom_index++, "%u", loaderparams.ram_size);

    prom_set(prom_buf, prom_index++, "modetty0");
    prom_set(prom_buf, prom_index++, "38400n8r");
    prom_set(prom_buf, prom_index++, NULL);

    rom_add_blob_fixed("prom", prom_buf, prom_size,
                       cpu_mips_kseg0_to_phys(NULL, ENVP_ADDR));

    g_free(prom_buf);
DB_PRINT("*************kernel_entry 0x%lx", kernel_entry);
    return kernel_entry;
}

static void ar71xx_mips_config(MIPSCPU *cpu)
{
    CPUMIPSState *env = &cpu->env;
    CPUState *cs = CPU(cpu);

    env->mvp->CP0_MVPConf0 |= ((smp_cpus - 1) << CP0MVPC0_PVPE) |
                         ((smp_cpus * cs->nr_threads - 1) << CP0MVPC0_PTC);
}

static void main_cpu_reset(void *opaque)
{
    MIPSCPU *cpu = opaque;
    CPUMIPSState *env = &cpu->env;

    cpu_reset(CPU(cpu));

    /* The bootloader does not need to be rewritten as it is located in a
       read only location. The kernel location and the arguments table
       location does not change. */
    if (loaderparams.kernel_filename) {
        env->CP0_Status &= ~(1 << CP0St_ERL);
    }

    ar71xx_mips_config(cpu);

    if (kvm_enabled()) {
        /* Start running from the bootloader we wrote to end of RAM */
        env->active_tc.PC = 0x40000000 + loaderparams.ram_low_size;
    }
}

static void create_cpu_without_cps(const char *cpu_type,
                                   qemu_irq *cbus_irq, qemu_irq *i8259_irq)
{
    CPUMIPSState *env;
    MIPSCPU *cpu;
    int i;

    for (i = 0; i < smp_cpus; i++) {
        cpu = MIPS_CPU(cpu_create(cpu_type));

        /* Init internal devices */
        cpu_mips_irq_init_cpu(cpu);
        cpu_mips_clock_init(cpu);
        qemu_register_reset(main_cpu_reset, cpu);
    }

    cpu = MIPS_CPU(first_cpu);
    env = &cpu->env;
    *i8259_irq = env->irq[2];
    *cbus_irq = env->irq[4];
}

static void create_cps(AR71xxState *s, const char *cpu_type,
                       qemu_irq *cbus_irq, qemu_irq *i8259_irq)
{
    Error *err = NULL;

    s->cps = MIPS_CPS(object_new(TYPE_MIPS_CPS));
    qdev_set_parent_bus(DEVICE(s->cps), sysbus_get_default());

    object_property_set_str(OBJECT(s->cps), cpu_type, "cpu-type", &err);
    object_property_set_int(OBJECT(s->cps), smp_cpus, "num-vp", &err);
    object_property_set_bool(OBJECT(s->cps), true, "realized", &err);
    if (err != NULL) {
        error_report("%s", error_get_pretty(err));
        exit(1);
    }

    sysbus_mmio_map_overlap(SYS_BUS_DEVICE(s->cps), 0, 0, 1);

    *i8259_irq = get_cps_irq(s->cps, 3);
    *cbus_irq = NULL;
}

static void mips_create_cpu(AR71xxState *s, const char *cpu_type,
                            qemu_irq *cbus_irq, qemu_irq *i8259_irq)
{
    if ((smp_cpus > 1) && cpu_supports_cps_smp(cpu_type)) {
        create_cps(s, cpu_type, cbus_irq, i8259_irq);
    } else {
        create_cpu_without_cps(cpu_type, cbus_irq, i8259_irq);
    }
}

static
void mips_ar71xx_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    ram_addr_t ram_low_size;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    pflash_t *fl;
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *sdram = g_new(MemoryRegion, 1);
    MemoryRegion *dram = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);
    MemoryRegion *ram_high = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MemoryRegion *bios_copy = g_new(MemoryRegion, 1);
    target_long bios_size = FLASH_SIZE;

    int64_t kernel_entry, bootloader_run_addr;
    PCIBus *pci_bus;
//    ISABus *isa_bus = NULL;
    qemu_irq *isa_irq;
    qemu_irq cbus_irq, i8259_irq;
//    int piix4_devfn;
//    I2CBus *smbus;
    DriveInfo *dinfo;
//    DriveInfo *hd[MAX_IDE_BUS * MAX_IDE_DEVS];
    int fl_idx = 0;
    int fl_sectors = bios_size >> 16;
    int be;
    DriveInfo *nand;
DeviceState *ds;
    Error *local_errp = NULL;
    AR71xxFPGAState *d;
  //  Chardev *chr;

    d = (AR71xxFPGAState *)g_malloc0(sizeof(AR71xxFPGAState));
    DeviceState *dev = qdev_create(NULL, TYPE_MIPS_AR71XX);
    AR71xxState *s = MIPS_AR71XX(dev);

    /* Throw exception when accessing invalid memory, i.e. memory > max memory emulated . Create an empty slot to
       emulate this feature. */
//    empty_slot_init(ram_size, 0x10);

    qdev_init_nofail(dev);

    /* create CPU */
    mips_create_cpu(s, machine->cpu_type, &cbus_irq, &i8259_irq);

    /* allocate RAM */
    if (ram_size > 2 * GiB) {
        error_report("Too much memory for this machine: %" PRId64 "MB,"
                     " maximum 2048MB", ram_size / MiB);
        exit(1);
    }
    /* register DRAM at 0x00000000 */
    memory_region_allocate_system_memory(dram, NULL, "mips_ar71xx.dram",
                                        ram_size);
//    memory_region_init_ram(dram, NULL, "mips_ar71xx.dram",
  //                                      ram_size, &error_fatal);
    memory_region_add_subregion(system_memory, 0x00000000LL, dram);

    /* register SRAM at 0x1d000000 */
    memory_region_init_ram(sram, NULL, "mips_ar71xx.sram",
                                        0x1000000, &error_fatal);
    memory_region_add_subregion(system_memory, 0x1d000000LL, sram);
    memory_region_init_ram(ram_high, NULL, "mips_ar71xx.flashram",
                                        ram_size, &error_fatal);
    memory_region_add_subregion(system_memory, 0xa0000000LL, ram_high);
#if 0 
    /* alias SRAM to DRAM */
        memory_region_init_alias(sram, NULL,
                                 "mips_ar71xx.sram",
                                 dram, 0,
                                 0x1000000);
        memory_region_add_subregion(system_memory, 0x1d000000LL,
                                    sram);
#endif
    memory_region_init_ram(bios, NULL, "u-boot.bin", BIOS_SIZE+0x200000,
                           &error_fatal);
    memory_region_set_readonly(bios, true);
    memory_region_init_alias(bios_copy, NULL, "u-boot.bin", bios,
                             0, BIOS_SIZE+0x200000);
    memory_region_init_alias(sdram, NULL, "u-boot.bin", bios,
                             0, BIOS_SIZE+0x200000);
    memory_region_add_subregion(system_memory, 0x1fc00000LL, bios);



#ifdef TARGET_WORDS_BIGENDIAN
    be = 1;
#else
    be = 0;
#endif

    /* FPGA */

    /* The CBUS UART is attached to the MIPS CPU INT2 pin, ie interrupt 11 */
    ar71xx_fpga_init(system_memory, FPGA_ADDRESS, cbus_irq, serial_hd(1));

    /* Load firmware in flash / BIOS. */
    dinfo = drive_get(IF_PFLASH, 0, fl_idx);

#ifdef DEBUG_BOARD_INIT

    if (dinfo) {
        DB_PRINT("%s: register BIOS\n", __func__);
//        BlockBackend *blk = blk_by_legacy_dinfo(dinfo);
        DB_PRINT("Register parallel flash %d size " TARGET_FMT_lx " at "
               "addr %08llx '%s' %x\n",
               fl_idx, bios_size, FLASH_ADDRESS,
               blk_name(blk), fl_sectors);

#endif
    fl = pflash_cfi01_register(FLASH_ADDRESS, NULL, "mips_ar71xx.bios",
                               BIOS_SIZE,
                               dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
                               65536, fl_sectors,
                               4, 0x00ad, 0x00da, 0x0000, 0x0000, be);
    bios = pflash_cfi01_get_memory(fl);
    fl_idx++;
    }
    if (kernel_filename) {
        ram_low_size = MIN(ram_size, 256 * MiB);
        /* For KVM we reserve 1MB of RAM for running bootloader */
        if (kvm_enabled()) {
            ram_low_size -= 0x100000;
            bootloader_run_addr = 0x40000000 + ram_low_size;
        } else {
            bootloader_run_addr = 0xbfc00000;
        }

        /* Write a small bootloader to the flash location. */
        loaderparams.ram_size = ram_size;
        loaderparams.ram_low_size = ram_low_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        kernel_entry = load_kernel();

        if (!cpu_supports_isa(machine->cpu_type, ISA_NANOMIPS32)) {
            write_bootloader(memory_region_get_ram_ptr(bios),
                             bootloader_run_addr, kernel_entry);
        } else {
            write_bootloader_nanomips(memory_region_get_ram_ptr(bios),
                                      bootloader_run_addr, kernel_entry);
        }
        if (kvm_enabled()) {
            /* Write the bootloader code @ the end of RAM, 1MB reserved */
  //          write_bootloader(memory_region_get_ram_ptr(ram_low_preio) +
     //                               ram_low_size,
        //                     bootloader_run_addr, kernel_entry);
        }
    } else {
        /* The flash region isn't executable from a KVM guest */
        if (kvm_enabled()) {
            error_report("KVM enabled but no -kernel argument was specified. "
                         "Booting from flash is not supported with KVM.");
            exit(1);
        }
        /* Load firmware from flash. */
        if (!dinfo) { 
            /* Load a BIOS image. */
            if (bios_name == NULL) {
                bios_name = BIOS_FILENAME;
            }
            filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
            if (filename) { DB_PRINT("biosize = 0x%x 0x%lx", bios_size, BIOS_SIZE);
                bios_size = load_image_targphys(filename, 0x1fc00000,
                                                BIOS_SIZE+0x200000);
                g_free(filename);
            } else {
                bios_size = -1;
            } DB_PRINT("biosize = 0x%x 0x%lx", bios_size, BIOS_SIZE);
            if ((bios_size < 0 || bios_size > BIOS_SIZE+0x200000) &&
                !kernel_filename && !qtest_enabled()) { 
                error_report("Could not load MIPS bios '%s', and no "
                             "-kernel argument was specified", bios_name);
                exit(1);
            }
        }
        /* In little endian mode the 32bit words in the bios are swapped,
           a neat trick which allows bi-endian firmware. */
#ifndef TARGET_WORDS_BIGENDIAN
        {
            uint32_t *end, *addr;
            const size_t swapsize = MIN(bios_size, 0x3e0000);
            addr = rom_ptr(FLASH_ADDRESS, swapsize);
            if (!addr) {
                addr = memory_region_get_ram_ptr(bios);
            }
            end = (void *)addr + swapsize;
            while (addr < end) {
                bswap32s(addr);
                addr++;
            }
        }
#endif
    }

    
    /* Map the BIOS at a 2nd physical location, as on the real board. */

    /* u-boot_mod Pepe2k */
#if 1  
    memory_region_add_subregion(system_memory, 0x1f000000LL, bios_copy);
    DB_PRINT("uboot MOD here*****");
    if (!rom_copy(memory_region_get_ram_ptr(bios_copy),
                  0x1f000000, BIOS_SIZE)) { 
        memcpy(memory_region_get_ram_ptr(bios_copy),
               memory_region_get_ram_ptr(bios), BIOS_SIZE);
    }

    /* u-boot DUMP real board --- bypass Wasp Bootrom until original wasp bootrom dump works */
#else 

    memory_region_add_subregion(system_memory, 0x00100000LL, sdram);
    DB_PRINT("uboot DUMP  here*****");
    if (!rom_copy(memory_region_get_ram_ptr(sdram),
                  0x00100000, BIOS_SIZE)) { 
        memcpy(memory_region_get_ram_ptr(sdram),
               memory_region_get_ram_ptr(bios), BIOS_SIZE);
    }


#endif
    /*
     * We have a circular dependency problem: pci_bus depends on isa_irq,
     * isa_irq is provided by i8259, i8259 depends on ISA, ISA depends
     * on piix4, and piix4 depends on pci_bus.  To stop the cycle we have
     * qemu_irq_proxy() adds an extra bit of indirection, allowing us
     * to resolve the isa_irq -> i8259 dependency after i8259 is initialized.
     */
    isa_irq = qemu_irq_proxy(&s->i8259, 16);

    /* Northbridge */
    pci_bus = gt64120_register(isa_irq);

    /* Southbridge */
//    ide_drive_get(hd, ARRAY_SIZE(hd));

//    piix4_devfn = piix4_init(pci_bus, &isa_bus, 80);

    /* Interrupt controller */
    /* The 8259 is attached to the MIPS CPU INT0 pin, ie interrupt 2 */
//    s->i8259 = i8259_init(isa_bus, i8259_irq);

  //  isa_bus_irqs(isa_bus, s->i8259);
 //   pci_piix4_ide_init(pci_bus, hd, piix4_devfn + 1);
   // pci_create_simple(pci_bus, piix4_devfn + 2, "piix4-usb-uhci");
//    smbus = piix4_pm_init(pci_bus, piix4_devfn + 3, 0x1100,
  //                        isa_get_irq(NULL, 9), NULL, 0, NULL);
//    pit = i8254_pit_init(isa_bus, 0x40, 0, NULL);
  //  i8257_dma_init(isa_bus, 0);
  //  mc146818_rtc_init(isa_bus, 2000, NULL);


    /* Super I/O: SMS FDC37M817 */
    //isa_create_simple(isa_bus, TYPE_FDC37M81X_SUPERIO);

    /* Network card */
    network_init(pci_bus);


    nand = drive_get(IF_MTD, 0, 0);

    ds = sysbus_create_simple("ath_nand", 0x1b000200u, NULL);
    d->nand = ds;
    const char param[] = "4F 4E 46 49 2 0 1C 0 1B 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 48 59 4E 49 58 20 20 20 20 20 20 20 48 32 37 55 32 47 38 46 32 43 54 52 2D 42 43 20 20 20 20 20 AD 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 8 0 0 40 0 0 2 0 0 10 0 40 0 0 0 0 8 0 0 1 23 1 28 0 1 5 1 0 0 4 0 1 1 4 0 0 0 0 0 0 0 0 0 0 0 0 0 A 1F 0 1F 0 BC 2 A 0 19 0 64 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 6E 68 0 0";
   ds = nand_init(nand ? blk_by_legacy_dinfo(nand) : NULL,
                         NAND_MFR_HYNIX, 0xda, param);
    object_property_set_link(OBJECT(d->nand),
                             OBJECT(ds),
                             "flash",
                             &local_errp);
    if (local_errp) {
        fprintf(stderr, "a369: Unable to set flash link for ATHNAND\n");
        abort();
    }
}

static int mips_ar71xx_sysbus_device_init(SysBusDevice *sysbusdev)
{
    return 0;
}

static void mips_ar71xx_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = mips_ar71xx_sysbus_device_init;
}

static const TypeInfo mips_ar71xx_device = {
    .name          = TYPE_MIPS_AR71XX,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AR71xxState),
    .class_init    = mips_ar71xx_class_init,
};

static void mips_ar71xx_machine_init(MachineClass *mc)
{
    mc->desc = "MIPS AR71xx";
    mc->init = mips_ar71xx_init;
    mc->block_default_type = IF_IDE;
    mc->max_cpus = 16;
    mc->is_default = 1;
#ifdef TARGET_MIPS64
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("20Kc");
#else
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("74Kf");
#endif
}

DEFINE_MACHINE("ar71xx", mips_ar71xx_machine_init)

static void mips_ar71xx_register_types(void)
{
    type_register_static(&mips_ar71xx_device);
}

type_init(mips_ar71xx_register_types)
