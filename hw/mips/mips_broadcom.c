/*
 * QEMU/mipssim emulation
 *
 * Emulates a very simple machine model similar to the one used by the
 * proprietary MIPS emulator.
 * 
 * Copyright (c) 2007 Thiemo Seufer
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
#include "qapi/error.h"
#include "qemu-common.h"
#include "cpu.h"
#include "hw/hw.h"
#include "hw/mips/mips.h"
#include "hw/mips/cpudevs.h"
#include "hw/char/serial.h"
#include "hw/isa/isa.h"
#include "net/net.h"
#include "sysemu/sysemu.h"
#include "hw/boards.h"
#include "hw/mips/bios.h"
#include "hw/loader.h"
#include "elf.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"
#include "qemu/error-report.h"
#include "sysemu/qtest.h"
#include "hw/block/flash.h"

#define BYTE_ACCESS_SIZE 1
#define HALFWORD_ACCESS_SIZE 2
#define WORD_ACCESS_SIZE 4


static struct _loaderparams {
    int ram_size;
    const char *kernel_filename;
    const char *kernel_cmdline;
    const char *initrd_filename;
} loaderparams;

typedef struct ResetData {
    MIPSCPU *cpu;
    uint64_t vector;
} ResetData;

static const int sector_len = 1 * 1024;

/* ChipID register.  */

#define TYPE_BCM_CHIP "bcm_chipid"
#define BCM_CHIP(obj) \
    OBJECT_CHECK(bcm_chipid_state, (obj), TYPE_BCM_CHIP)

typedef struct {
    SysBusDevice parent_obj;

    qemu_irq irq;
    MemoryRegion iomem;
    uint32_t chip_id;
    uint32_t mcs;
    uint32_t mdr;
    uint32_t mtpr;
    uint32_t mimr;
    uint32_t mris;
    uint32_t mcr;
} bcm_chipid_state;

static uint64_t bcm_chipid_read(void *opaque, hwaddr offset,
                                   unsigned size)
{
    bcm_chipid_state *s = (bcm_chipid_state *)opaque;

    switch (offset) {
    case 0x00: /* MSA */
        return s->chip_id;
    case 0x04: /* MCS */
        /* We don't emulate timing, so the controller is never busy.  */
        return s->mcs;
    default:
        hw_error("%s: Bad offset 0x%x\n", __func__, (int)offset);
        return 0;
    }
}

static void bcm_chipid_write(void *opaque, hwaddr offset,
                                uint64_t value, unsigned size)
{
    bcm_chipid_state *s = (bcm_chipid_state *)opaque;

    switch (offset) {
    case 0x00: /* MSA */
        //s->chip_id = value;
        break;
    case 0x04: /* MCS */
        break;
    case 0x08: /* MDR */
        s->mdr = value & 0xff;
        break;
    default:
        hw_error("bcm_chipid_write: Bad offset 0x%x\n",
                  (int)offset);
    }
}

static void bcm_chipid_reset(bcm_chipid_state *s)
{
    s->chip_id = 0x5357;
    s->mcs = 0;
    s->mdr = 0;
    s->mtpr = 1;
    s->mimr = 0;
    s->mris = 0;
    s->mcr = 0;
}

static const MemoryRegionOps bcm_chipid_ops = {
    .read = bcm_chipid_read,
    .write = bcm_chipid_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
	.impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    }
};

static const VMStateDescription vmstate_bcm_chipid = {
    .name = "bcm-chipid",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(chip_id, bcm_chipid_state),
        VMSTATE_UINT32(mcs, bcm_chipid_state),
        VMSTATE_UINT32(mdr, bcm_chipid_state),
        VMSTATE_UINT32(mtpr, bcm_chipid_state),
        VMSTATE_UINT32(mimr, bcm_chipid_state),
        VMSTATE_UINT32(mris, bcm_chipid_state),
        VMSTATE_UINT32(mcr, bcm_chipid_state),
        VMSTATE_END_OF_LIST()
    }
};

static void bcm_chipid_init(Object *obj)
{
    //DeviceState *dev = DEVICE(obj);
    bcm_chipid_state *s = BCM_CHIP(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    sysbus_init_irq(sbd, &s->irq);

    memory_region_init_io(&s->iomem, obj, &bcm_chipid_ops, s,
                          "chipid", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    bcm_chipid_reset(s);
}

static void bcm_chipid_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_bcm_chipid;
}

static const TypeInfo bcm_chipid_info = {
    .name          = TYPE_BCM_CHIP,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(bcm_chipid_state),
    .instance_init = bcm_chipid_init,
    .class_init    = bcm_chipid_class_init,
};

static int64_t load_kernel(void)
{
    int64_t entry, kernel_high;
    long kernel_size;
    long initrd_size;
    ram_addr_t initrd_offset;
    int big_endian;

#ifdef TARGET_WORDS_BIGENDIAN
    big_endian = 1;
#else
    big_endian = 0;
#endif

    kernel_size = load_elf(loaderparams.kernel_filename, cpu_mips_kseg0_to_phys,
                           NULL, (uint64_t *)&entry, NULL,
                           (uint64_t *)&kernel_high, big_endian,
                           EM_MIPS, 1, 0);
    if (kernel_size >= 0) {
        if ((entry & ~0x7fffffffULL) == 0x80000000)
            entry = (int32_t)entry;
    } else {
        fprintf(stderr, "qemu: could not load kernel '%s'\n",
                loaderparams.kernel_filename);
        exit(1);
    }
    printf("kernel_size: %ld\n", kernel_size);

    /* load initrd */
    initrd_size = 0;
    initrd_offset = 0;
    if (loaderparams.initrd_filename) {
        initrd_size = get_image_size (loaderparams.initrd_filename);
        if (initrd_size > 0) {
            initrd_offset = (kernel_high + ~INITRD_PAGE_MASK) & INITRD_PAGE_MASK;
            if (initrd_offset + initrd_size > loaderparams.ram_size) {
                fprintf(stderr,
                        "qemu: memory too small for initial ram disk '%s'\n",
                        loaderparams.initrd_filename);
                exit(1);
            }
            initrd_size = load_image_targphys(loaderparams.initrd_filename,
                initrd_offset, loaderparams.ram_size - initrd_offset);
        }
        if (initrd_size == (target_ulong) -1) {
            fprintf(stderr, "qemu: could not load initial ram disk '%s'\n",
                    loaderparams.initrd_filename);
            exit(1);
        }
    }
    return entry;
}

static void main_cpu_reset(void *opaque)
{
    ResetData *s = (ResetData *)opaque;
    CPUMIPSState *env = &s->cpu->env;

    cpu_reset(CPU(s->cpu));
    env->active_tc.PC = s->vector & ~(target_ulong)1;
    if (s->vector & 1) {
        env->hflags |= MIPS_HFLAG_M16;
    }
}

/*
static void mipschip_init(hwaddr base, qemu_irq irq)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_create(NULL, "bcm_chipid");
    //qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);

    s = SYS_BUS_DEVICE(dev);
    sysbus_connect_irq(s, 0, irq);
	sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);
}
*/

static void mipsnet_init(int base, qemu_irq irq, NICInfo *nd)
{
    DeviceState *dev;
    SysBusDevice *s;

    dev = qdev_create(NULL, "mipsnet");
    qdev_set_nic_properties(dev, nd);
    qdev_init_nofail(dev);

    s = SYS_BUS_DEVICE(dev);
    sysbus_connect_irq(s, 0, irq);
    memory_region_add_subregion(get_system_io(),
                                base,
                                sysbus_mmio_get_region(s, 0));
}

static void
bcm_router_init(MachineState *machine)
{
    ram_addr_t ram_size = machine->ram_size;
    const char *kernel_filename = machine->kernel_filename;
    const char *kernel_cmdline = machine->kernel_cmdline;
    const char *initrd_filename = machine->initrd_filename;
    char *filename;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *isa = g_new(MemoryRegion, 1);
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *bios = g_new(MemoryRegion, 1);
    MIPSCPU *cpu;
    CPUMIPSState *env;
    ResetData *reset_info;
	DriveInfo *dinfo;
    int bios_size;
	uint32_t flash_size = 0x20000; // 0x1000000
	int big_endian;
	//uint64_t pagemask;

#ifdef TARGET_WORDS_BIGENDIAN
    big_endian = 1;
#else
    big_endian = 0;
#endif

    /* Init CPUs. */
    cpu = MIPS_CPU(cpu_create(machine->cpu_type));
    env = &cpu->env;

    reset_info = g_malloc0(sizeof(ResetData));
    reset_info->cpu = cpu;
    reset_info->vector = env->active_tc.PC;
    qemu_register_reset(main_cpu_reset, reset_info);

    /* Allocate RAM. */
    memory_region_allocate_system_memory(ram, NULL, "bcm.ram",
                                         ram_size);
    memory_region_init_ram(bios, NULL, "bcm.bios", BIOS_SIZE,
                           &error_fatal);
//    vmstate_register_ram_global(bios);
    memory_region_set_readonly(bios, true);

    memory_region_add_subregion(address_space_mem, 0, ram);
	
	/* Map the BIOS / boot exception handler. */
    memory_region_add_subregion(address_space_mem, 0x1fc00000LL, bios);
    /* Load a BIOS / boot exception handler image. */
    if (bios_name == NULL)
        bios_name = BIOS_FILENAME;
    filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
    if (filename) {
        bios_size = load_image_targphys(filename, 0x1fc00000LL, BIOS_SIZE);
        g_free(filename);
    } else {
        bios_size = -1;
    }
    if ((bios_size < 0 || bios_size > BIOS_SIZE) &&
        !kernel_filename && !qtest_enabled()) {
        /* Bail out if we have neither a kernel image nor boot vector code. */
        error_report("Could not load MIPS bios '%s', and no "
                     "-kernel argument was specified", bios_name);
        //exit(1);
    } else {
        /* We have a boot vector start address. */
        env->active_tc.PC = (target_long)(int32_t)0xbfc00000;
    }

    if ((bios_size < 0) && !kernel_filename) {
		dinfo = drive_get(IF_PFLASH, 0, 0);
		if (!dinfo && !qtest_enabled()) {
			fprintf(stderr, "A flash image must big_endian given with the "
					"'pflash' parameter\n");
			exit(1);
		} else {
			if (!pflash_cfi01_register(0xbfc00000, NULL, "router.flash",
				flash_size,
				dinfo ? blk_by_legacy_dinfo(dinfo) : NULL,
				sector_len, flash_size / sector_len,
				4, 0, 0, 0, 0, big_endian)) {
				fprintf(stderr, "qemu: Error registering flash memory.\n");
				exit(1);
			}
		}
	}

    if (kernel_filename) {
        loaderparams.ram_size = ram_size;
        loaderparams.kernel_filename = kernel_filename;
        loaderparams.kernel_cmdline = kernel_cmdline;
        loaderparams.initrd_filename = initrd_filename;
        reset_info->vector = load_kernel();
    }
	/*MemoryRegionSection mrs = memory_region_find(address_space_mem,
						0x80700000, WORD_ACCESS_SIZE);
    MemoryRegion *flash_alias = g_new(MemoryRegion, 1);
    memory_region_init_alias(
            flash_alias,
            NULL,
            "router.flash.alias",
            mrs.mr,
            0,
            flash_size);
    memory_region_add_subregion(address_space_mem, 0, flash_alias);*/

    /* Init CPU internal devices. */
    cpu_mips_irq_init_cpu(cpu);
    cpu_mips_clock_init(cpu);
	/*pagemask = TARGET_PAGE_MASK;
	printf("TARGET_PAGE_MASK: %lx\n", pagemask);*/

    /* Register 64 KB of ISA IO space at 0x1fd00000. */
    memory_region_init_alias(isa, NULL, "isa_mmio",
                             get_system_io(), 0, 0x00010000);
    memory_region_add_subregion(get_system_memory(), 0x1fd00000, isa);

	sysbus_create_simple(TYPE_BCM_CHIP, 0x18000000, NULL);
	//mipschip_init(0xb8000000, NULL);

    /* A single 16450 sits at offset 0x3f8. It is attached to
       MIPS CPU INT2, which is interrupt 4. */
    if (serial_hd(0))
        serial_init(0x3f8, env->irq[4], 115200, serial_hd(0),
                    get_system_io());

    if (nd_table[0].used)
        /* MIPSnet uses the MIPS CPU INT0, which is interrupt 2. */
        mipsnet_init(0x4200, env->irq[2], &nd_table[0]);
}

static void bcm_register_types(void)
{
    type_register_static(&bcm_chipid_info);
    //type_register_static(&stellaris_gptm_info);
    //type_register_static(&stellaris_adc_info);
}

type_init(bcm_register_types)

static void bcm_router_machine_init(MachineClass *mc)
{
    mc->desc = "Router based on BCM5358";
    mc->init = bcm_router_init;
#ifdef TARGET_MIPS64
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("5Kf");
#else
    mc->default_cpu_type = MIPS_CPU_TYPE_NAME("74Kf");
#endif
}

DEFINE_MACHINE("bcm_router", bcm_router_machine_init)
