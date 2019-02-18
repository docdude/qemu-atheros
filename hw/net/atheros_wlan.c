/**
 * QEMU WLAN device emulation
 * 
 * Copyright (c) 2008 Clemens Kolbitsch
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a 
copy
 * of this software and associated documentation files (the "Software"), to 
deal
 * in the Software without restriction, including without limitation the 
rights
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * Modifications:
 *  2008-February-24  Clemens Kolbitsch :
 *                                  New implementation based on ne2000.c
 *
 */

#include "qemu/osdep.h"
#include "hw/hw.h"
#include "hw/pci/pci.h"
#include "net/net.h"
//#include "pc.h"
//#include "migration/register.h"
#include "hw/loader.h"
//#include "sysemu/dma.h"
#include "sysemu/sysemu.h"
#include "trace.h"
/*
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/sem.h>
#include <sys/mman.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <signal.h>

#include <time.h>
#include <sys/time.h>
*/
/*
 * PCI and EEPROM definitions
 */
#include "hw/net/atheros_wlan.h"
#include "hw/net/atheros_wlan_io.h"
#include "hw/net/atheros_wlan_ap.h"
#include "hw/net/atheros_wlan_eeprom.h"

/*
 * MadWifi OPENHAL atheros constants
 */
#include "hw/net/ath5k_hw.h"
#include "hw/net/ath5kreg.h"
//#include "hw/net/ath5k.h"


static void Atheros_WLAN_reset(Atheros_WLANState *s)
{
       DEBUG_PRINT(("reset\n"));

       /*
        * Restore mac address
        */

       memcpy(s->macaddr, s->conf.macaddr.a, 6);

       /*
        * data from my local AR5212 device
        */
       SET_MEM_L(s->mem, 12, 0);
       SET_MEM_L(s->mem, AR5K_SREV, 86);
       SET_MEM_L(s->mem, AR5K_PCICFG, 0x00010014);
       SET_MEM_L(s->mem, AR5K_PHY_CHIP_ID, 65);
       SET_MEM_L(s->mem, AR5K_SLEEP_CTL, 0x00010000);
       SET_MEM_L(s->mem, 0x9820, 0x02020200);

       Atheros_WLAN_update_irq(s);
}

static void Atheros_WLAN_setup_type(PCIDevice *pci_dev)
{
       // create buffer large enough to
       // do all checks
       char *device_name;
       char nd_model[128];
       uint8_t *pci_conf;
       PCIAtheros_WLANState *d = PCI_ATHEROS_WLAN(pci_dev);
       Atheros_WLANState *s;
       s = &d->Atheros_WLAN;

       NICInfo *nd = NULL;
       device_name = nd_model;
       pci_conf = pci_dev->config;

       int n;
       for(n = 0; n < nb_nics; n++) {
            nd = &nd_table[n];
       }
       snprintf(nd_model, sizeof(nd_model), "%s", nd->model);


       // skip "atheros_wlan"
       // if it had not been part of nd->model, this
       // module would not be loaded anyways!!
       device_name += 12;

       DEBUG_PRINT_AP((" * Loading virtual wlan-pci device %s...\n", nd->model));

       if (strncmp(device_name, "_winxp", 6) == 0)
       {
               s->device_driver_type = WINXP_DRIVER;
               DEBUG_PRINT_AP((" * Make sure you are using a MS Windows driver!!\n"));

               // skip "_winxp"
               device_name += 6;
       }
       else if (strncmp(device_name, "_linux", 6) == 0)
       {
               s->device_driver_type = LINUX_DRIVER;
               DEBUG_PRINT_AP((" * Make sure you are using a MadWifi driver!!\n"));

               // skip "_linux"
               device_name += 6;
       }
       else
       {
               s->device_driver_type = LINUX_DRIVER;
               DEBUG_PRINT_AP((" * Unknown driver type '%s'... defaulting to Linux... Make sure you are using a MadWifi driver!!\n", nd->model));
       }

       if (strncmp(device_name, "_HPW400", 7) == 0)
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_HPW400;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_HPW400);

               memcpy(pci_conf, Atheros_WLAN_pci_config_HPW400, 256);

               DEBUG_PRINT_AP((" * Using EEPROM and device configuration of HP W400!!\n"));

               // skip "_HPW400"
               device_name += 7;
       }
       else if (strncmp(device_name, "_MacBook", 8) == 0)
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_MacBook;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_MacBook);

               memcpy(pci_conf, Atheros_WLAN_pci_config_MacBook, 256);
               
               DEBUG_PRINT_AP((" * Using EEPROM and device configuration of Mac Book!!\n"));

               // skip "_MacBook"
               device_name += 8;
       }
       else if (strncmp(device_name, "_AR5001XPlus", 12) == 0)
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_HPW400;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_HPW400);

               memcpy(pci_conf, Atheros_WLAN_pci_config_AR5001XPlus, 256);
               
               DEBUG_PRINT_AP((" * Using EEPROM and device configuration of AR5001X+ (e.g. Toshiba A100)!!\n"));

               // skip "_AR5001XPlus"
               device_name += 12;
       }
       else if (strncmp(device_name, "_John", 5) == 0)
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_HPW400;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_HPW400);

               memcpy(pci_conf, Atheros_WLAN_pci_config_JOHN, 256);
               
               DEBUG_PRINT_AP((" * Using EEPROM and device configuration of John!!\n"));

               // skip "_John"
               device_name += 5;
       }
       else if (strncmp(device_name, "_TPLinkWN651G", 13) == 0)
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_HPW400;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_HPW400);

               memcpy(pci_conf, Atheros_WLAN_pci_config_TP_Link_WN651G, 64);
               
               DEBUG_PRINT_AP((" * Using EEPROM and device configuration of TP-Link WN651G!!\n"));

               // skip "_TPLinkWN651G"
               device_name += 13;
       }
       else
       {
               s->eeprom_data = (u_int32_t*)Atheros_WLAN_eeprom_data_HPW400;
               s->eeprom_size = sizeof(Atheros_WLAN_eeprom_data_HPW400);

               memcpy(pci_conf, Atheros_WLAN_pci_config_HPW400, 256);

               DEBUG_PRINT_AP((" * Unknown EEPROM type '%s'... defaulting to HP W400!!\n", 
nd->model));       }
}


static int Atheros_WLAN_pre_save(void* opaque)
{
   //    int i;
    //   uint32_t direct_value;
       Atheros_WLANState *s = opaque;
       s->Atheros_WLAN_mmio_io_addr_dummy = 0;
/*
       if (s->pci_dev)
       {
               pci_device_save(s->pci_dev, f);
       }

       qemu_put_be32s(f, (uint32_t *)&s->device_driver_type);

       qemu_put_buffer(f, s->ipaddr, 4);
       qemu_put_buffer(f, s->macaddr, 6);

       qemu_put_buffer(f, s->ap_ipaddr, 4);
       qemu_put_buffer(f, s->ap_macaddr, 6);


       qemu_put_be32s(f, &s->interrupt_p_mask);

       for (i=0; i<5; qemu_put_be32s(f, &s->interrupt_s_mask[i++]));
       qemu_put_8s(f, &s->interrupt_enabled);

       qemu_put_be32s(f, &s->current_frequency);
*/
   //    direct_value = (uint32_t)s->receive_queue_address;
//       qemu_put_be32s(f, &direct_value);
//       qemu_put_be32s(f, &s->receive_queue_count);

 //      qemu_put_be32s(f, &s->transmit_queue_size);
 //      for (i=0; i<16; i++)
   //    {
            //   qemu_put_8s(f, &s->transmit_queue_enabled[i]);
      //         direct_value = (uint32_t)s->transmit_queue_address[i];
            //   qemu_put_be32s(f, &direct_value);
             //  qemu_put_be32s(f, (const uint32_t *)&s->transmit_queue_processed[i]);
     //  }

     //  qemu_put_be32s(f, (uint32_t*)&s->ap_state);
      // qemu_put_be32s(f, &s->inject_sequence_number);

      // qemu_put_buffer(f, (uint8_t *)s->mem, Atheros_WLAN_MEM_SIZE);
return 0;
}

static int Atheros_WLAN_post_load(void* opaque, int version_id)
{
//       int i, ret;
//       uint32_t direct_value;
 //      int i;
       Atheros_WLANState *s = opaque;

       // everyone has version 3... and the pci
       // stuff should be there as well, I think
       //
       // let's just claim this has been around
       // for quite some time ;-)
       if (version_id != 3)
               return -EINVAL;
/*
       if (s->pci_dev && version_id >= 3) {
               ret = pci_device_load(s->pci_dev, f);
               if (ret < 0)
                       return ret;
       }

       qemu_get_be32s(f, (uint32_t *)&s->device_driver_type);

       qemu_get_buffer(f, s->ipaddr, 4);
       qemu_get_buffer(f, s->macaddr, 6);

       qemu_get_buffer(f, s->ap_ipaddr, 4);
       qemu_get_buffer(f, s->ap_macaddr, 6);


       qemu_get_be32s(f, &s->interrupt_p_mask);
       for (i=0; i<5; qemu_get_be32s(f, &s->interrupt_s_mask[i++]));
       qemu_get_8s(f, &s->interrupt_enabled);

       qemu_get_be32s(f, &s->current_frequency);
       qemu_get_be32s(f, &direct_value);

       s->receive_queue_address = (uint32_t*)&direct_value;

       qemu_get_be32s(f, &s->receive_queue_count);

       qemu_get_be32s(f, &s->transmit_queue_size);

       for (i=0; i<16; i++)
       {
               //qemu_get_8s(f, &s->transmit_queue_enabled[i]);
               //qemu_get_be32s(f, &direct_value);
               s->transmit_queue_address[i] = (uint32_t*)&direct_value;
               //qemu_get_be32s(f, (uint32_t*)&s->transmit_queue_processed[i]);
       }
*/
       //qemu_get_be32s(f, (uint32_t*)&s->ap_state);
       //qemu_get_be32s(f, &s->inject_sequence_number);

      // qemu_get_buffer(f, (uint8_t *)s->mem, Atheros_WLAN_MEM_SIZE);


       s->inject_timer_running = 0;
       s->inject_queue_size = 0;
       s->inject_queue = NULL;

       return 0;
}

const VMStateDescription vmstate_Atheros_WLAN = {
    .name = "atheros_wlan",
    .version_id = 3,
    .minimum_version_id = 0,
    .pre_save = Atheros_WLAN_pre_save,
    .post_load = Atheros_WLAN_post_load,
    .fields = (VMStateField[]) {
	VMSTATE_INT32(Atheros_WLAN_mmio_io_addr_dummy, Atheros_WLANState),
	VMSTATE_INT32(device_driver_type, Atheros_WLANState),
        VMSTATE_BUFFER(ipaddr, Atheros_WLANState),
        VMSTATE_BUFFER(macaddr, Atheros_WLANState),
        VMSTATE_BUFFER(ap_ipaddr, Atheros_WLANState),
        VMSTATE_BUFFER(ap_macaddr, Atheros_WLANState),
        VMSTATE_UINT32(interrupt_p_mask, Atheros_WLANState),
        VMSTATE_UINT32_ARRAY(interrupt_s_mask, Atheros_WLANState, 5),

        VMSTATE_UINT8(interrupt_enabled, Atheros_WLANState),
	VMSTATE_INT32(access_semaphore, Atheros_WLANState),
	VMSTATE_UINT32_ARRAY(current_frequency_partial_data, Atheros_WLANState, 2),
        VMSTATE_UINT32(current_frequency, Atheros_WLANState),
	VMSTATE_INT32(do_bswp, Atheros_WLANState),
        VMSTATE_POINTER(receive_queue_address, Atheros_WLANState, 3, vmstate_info_uint32, uint32_t *),
        VMSTATE_UINT32(receive_queue_count, Atheros_WLANState),

        VMSTATE_UINT32(transmit_queue_size, Atheros_WLANState),
        VMSTATE_BUFFER(transmit_queue_enabled, Atheros_WLANState),

        VMSTATE_ARRAY_OF_POINTER(transmit_queue_address, Atheros_WLANState, 16, 3, vmstate_info_uint32, uint32_t *),
        VMSTATE_INT32_ARRAY(transmit_queue_processed, Atheros_WLANState, 16),
        VMSTATE_UINT32_ARRAY(mem, Atheros_WLANState, Atheros_WLAN_MEM_SIZE / 4),
        VMSTATE_INT32(eeprom_size, Atheros_WLANState),
 //       VMSTATE_POINTER(eeprom_data, Atheros_WLANState, 3, vmstate_info_uint32, u_int32_t *),
 //      u_int32_t *eeprom_data;

        VMSTATE_INT32(ap_state, Atheros_WLANState),
        VMSTATE_INT32(inject_timer_running, Atheros_WLANState),
        VMSTATE_UINT32(inject_sequence_number, Atheros_WLANState),
        VMSTATE_TIMER_PTR(beacon_timer, Atheros_WLANState),
        VMSTATE_TIMER_PTR(inject_timer, Atheros_WLANState),

        VMSTATE_INT32(inject_queue_size, Atheros_WLANState),
//	VMSTATE_STRUCT_POINTER(inject_queue, Atheros_WLANState, vmstate_info_uint32, mac80211_frame),

//        VMSTATE_UNUSED(4), /* was irq */

        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_pci_Atheros_WLAN = {
    .name = "atheros_wlan",
    .version_id = 3,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_PCI_DEVICE(parent_obj, PCIAtheros_WLANState),
        VMSTATE_STRUCT(Atheros_WLAN, PCIAtheros_WLANState, 1, vmstate_Atheros_WLAN, Atheros_WLANState),
        VMSTATE_END_OF_LIST()
    }
};


static void pci_Atheros_WLAN_realize(PCIDevice *pci_dev, Error **errp )
{
       PCIAtheros_WLANState *d = PCI_ATHEROS_WLAN(pci_dev);
       Atheros_WLANState *s;
       s = &d->Atheros_WLAN;


       /*
        * currently, we have to use this mac-address.
        * it is hardcoded in the eeprom/io-stuff
        */
       qemu_macaddr_default_if_unset(&s->conf.macaddr);
       s->conf.macaddr.a[0] = 0x00;
       s->conf.macaddr.a[1] = 0x11;
       s->conf.macaddr.a[2] = 0x0a;
       s->conf.macaddr.a[3] = 0x80;
       s->conf.macaddr.a[4] = 0x2e;
       s->conf.macaddr.a[5] = 0x9e;

//	d =(PCIAtheros_WLANState*)pci_register_device(bus,"Atheros_WLAN",sizeof(PCIAtheros_WLANState),devfn,  NULL,NULL);

//d =(PCIAtheros_WLANState*)do_pci_register_device(pci_dev,bus,"Atheros_WLAN",devfn, NULL);
   //    s = &d->Atheros_WLAN;

       // s->irq = 9; /* PCI interrupt */
//       s->irq = d->dev.irq[0];
//       s->pci_dev = (PCIDevice *)d;
       s->irq = pci_allocate_irq(pci_dev);
       s->pending_interrupts = NULL;
       memcpy(s->macaddr, s->conf.macaddr.a, 6);
       s->dma_opaque = pci_dev;
       Atheros_WLAN_setup_type(pci_dev);

       Atheros_WLAN_setup_io(d);
       pci_register_bar(pci_dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->mmio);
       Atheros_WLAN_setup_ap(DEVICE(pci_dev), s);

       Atheros_WLAN_reset(s);
       /* TODO: we don't support multiple instance yet!! */
 //      register_savevm_live(NULL, "Atheros_WLAN", 0, 3, &savevm_Atheros_WLAN, s);
//printf(" ********** I AM HERE!!!!!\n");
  //     Atheros_WLAN_reset(nd, s);
 //   s->nic = qemu_new_nic(&net_Atheros_WLAN_info, &s->conf,
  //                        object_get_typename(OBJECT(pci_dev)), pci_dev->qdev.id, s);
   // qemu_format_nic_info_str(qemu_get_queue(s->nic), s->conf.macaddr.a);
}

static void pci_reset(DeviceState *dev)
{
    PCIAtheros_WLANState *d = PCI_ATHEROS_WLAN(dev);

    Atheros_WLAN_reset(&d->Atheros_WLAN);
}

static void Atheros_WLAN_instance_init(Object *obj)
{
   // PCIDevice *pci_dev = PCI_DEVICE(obj);
 //   PCIAtheros_WLANState *d = DO_UPCAST(PCIAtheros_WLANState, dev, pci_dev);
    PCIAtheros_WLANState *d = PCI_ATHEROS_WLAN(obj);
   // PCNetState *s = &d->state;
    Atheros_WLANState *s = &d->Atheros_WLAN;

    device_add_bootindex_property(obj, &s->conf.bootindex,
                                  "bootindex", "/ethernet-phy@0",
                                  DEVICE(obj), NULL);
}



static Property Atheros_WLAN_properties[] = {
    DEFINE_NIC_PROPERTIES(PCIAtheros_WLANState, Atheros_WLAN.conf),
    DEFINE_PROP_END_OF_LIST(),
};

static void pci_Atheros_WLAN_uninit(PCIDevice *dev)
{
    PCIAtheros_WLANState *d = PCI_ATHEROS_WLAN(dev);

    qemu_free_irq(d->Atheros_WLAN.irq);
    timer_del(d->Atheros_WLAN.beacon_timer);
    timer_free(d->Atheros_WLAN.beacon_timer);
    timer_del(d->Atheros_WLAN.inject_timer);
    timer_free(d->Atheros_WLAN.inject_timer);
    qemu_del_nic(d->Atheros_WLAN.nic);
}



static void Atheros_WLAN_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    PCIDeviceClass *k = PCI_DEVICE_CLASS(klass);

    k->realize = pci_Atheros_WLAN_realize;

    k->exit = pci_Atheros_WLAN_uninit;
    k->romfile = "efi-ne2k_pci.rom",
    k->vendor_id = PCI_VENDOR_ID_ATHEROS;
    k->device_id = PCI_DEVICE_ID_ATHEROS_AR5212;
 //   k->revision = 0x10;
    k->class_id = PCI_CLASS_NETWORK_ETHERNET;
    dc->reset = pci_reset;
    dc->desc = "Atheros WLAN";
    dc->vmsd = &vmstate_pci_Atheros_WLAN;
    dc->props = Atheros_WLAN_properties;
    dc->user_creatable = true;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);

}

static const TypeInfo Atheros_WLAN_info = {
    .name          = TYPE_PCI_ATHEROS_WLAN,
    .parent        = TYPE_PCI_DEVICE,
    .instance_size = sizeof(PCIAtheros_WLANState),
    .class_init    = Atheros_WLAN_class_init,
    .instance_init = Atheros_WLAN_instance_init,
    .interfaces = (InterfaceInfo[]) {
        { INTERFACE_CONVENTIONAL_PCI_DEVICE },
        { },
    },
};

#if 0 //to do for eeprom versions
typedef struct ATHEROSWLANInfo {
    const char *name;
    uint16_t   device_id;
    uint8_t    revision;
 //   uint16_t   phy_id2;
} ATHEROSWLANInfo;

static const ATHEROSWLANInfo atheros_wlan_devices[] = {
    {
        .name      = "atheros_wlan",
        .device_id = 0x168c,
        .revision  = 0x03,
//        .phy_id2   = E1000_PHY_ID2_8254xx_DEFAULT,
    },
    {
        .name      = "atheros_wlan_linux_HPW400",
        .device_id = 0x168c,
        .revision  = 0x03,
 //       .phy_id2   = E1000_PHY_ID2_82544x,
    },
    {
        .name      = "atheros_wlan_linux_JOHN",
        .device_id = 0x168c,
        .revision  = 0x03,
  //      .phy_id2   = E1000_PHY_ID2_8254xx_DEFAULT,
    },
};
#endif
static void pci_Atheros_WLAN_register_types(void)
{
    type_register_static(&Atheros_WLAN_info);
}

type_init(pci_Atheros_WLAN_register_types)

