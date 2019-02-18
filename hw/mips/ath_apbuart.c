/*
 * QEMU GRLIB APB UART Emulator
 *
 * Copyright (c) 2010-2011 AdaCore
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
#include "hw/sysbus.h"
#include "chardev/char-fe.h"

//#include "trace.h"
//#define DEBUG_UART
#ifdef DEBUG_UART
#define DB_PRINT(...) do { \
    fprintf(stderr,  "[**%s**] ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0);
#else
    #define DB_PRINT(...)
#endif

#define UART_REG_SIZE 20     /* Size of memory mapped registers */

/* UART data register fields */
#define UART_RX_CSR               (1 <<  8)
#define UART_TX_CSR               (1 <<  9)

/* UART status register fields */
#define UART_DATA_READY           (1 <<  0)
#define UART_TRANSMIT_SHIFT_EMPTY (1 <<  1)
#define UART_TRANSMIT_FIFO_EMPTY  (1 <<  2)
#define UART_BREAK_RECEIVED       (1 <<  3)
#define UART_OVERRUN              (1 <<  4)
#define UART_PARITY_ERROR         (1 <<  5)
#define UART_FRAMING_ERROR        (1 <<  6)
#define UART_TRANSMIT_FIFO_HALF   (1 <<  7)
#define UART_RECEIVE_FIFO_HALF    (1 <<  8)
#define UART_TRANSMIT_FIFO_FULL   (1 <<  9)
#define UART_RECEIVE_FIFO_FULL    (1 << 10)

/* UART control register fields */
#define UART_RECEIVE_ENABLE          (1 <<  0)
#define UART_TRANSMIT_ENABLE         (1 <<  1)
#define UART_RECEIVE_INTERRUPT       (1 <<  2)
#define UART_TRANSMIT_INTERRUPT      (1 <<  3)
#define UART_PARITY_SELECT           (1 <<  4)
#define UART_PARITY_ENABLE           (1 <<  5)
#define UART_FLOW_CONTROL            (1 <<  6)
#define UART_LOOPBACK                (1 <<  7)
#define UART_EXTERNAL_CLOCK          (1 <<  8)
#define UART_RECEIVE_FIFO_INTERRUPT  (1 <<  9)
#define UART_TRANSMIT_FIFO_INTERRUPT (1 << 10)
#define UART_FIFO_DEBUG_MODE         (1 << 11)
#define UART_OUTPUT_ENABLE           (1 << 12)
#define UART_FIFO_AVAILABLE          (1 << 31)

/* Memory mapped register offsets */
#define UARTDATA       0x00
#define UARTCS     0x04
#define UARTCLOCK    0x08
#define UARTINT     0x0C  /* not supported */
#define UARTINTEN 0x10  /* not supported */

#define FIFO_LENGTH 4

#define TYPE_ATH_APB_UART "ath,apbuart"
#define ATH_APB_UART(obj) \
    OBJECT_CHECK(UART, (obj), TYPE_ATH_APB_UART)

typedef struct UART {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    CharBackend chr;

    /* registers */
    uint32_t status;
    uint32_t control;
    uint32_t clk;
    uint32_t intr;
    uint32_t intren;
    uint32_t csr;

    /* FIFO */
    char buffer[FIFO_LENGTH];
    int  len;
    int  current;
} UART;

static int uart_data_to_read(UART *uart)
{
    return uart->current < uart->len;
}

static int uart_pop(UART *uart)
{
    unsigned int ret=0;
     
    if (uart->len == 0) {
        uart->csr = UART_TX_CSR;
 // DB_PRINT ("buffer: 0x%x uart->len: %d uart->csr: 0x%x\n", uart->buffer[uart->current++], uart->len, uart->csr);
        return uart->csr;
    }
    if (uart->len > 0 && (uart->csr & UART_RX_CSR)) {
       ret = (uart->buffer[uart->current++] | UART_RX_CSR);
    }
 DB_PRINT ("buffer: 0x%x ret: 0x%x uart->current: %d uart->len: %d\n", uart->buffer[uart->current], ret, uart->current, uart->len);
    if (uart->current >= uart->len) {
        /* Flush */
        uart->len     = 0;
        uart->current = 0;
    }

    if (uart_data_to_read(uart)) {
        uart->csr = UART_RX_CSR;
        return uart->csr;
    }

    return ret;
}

static void uart_add_to_fifo(UART          *uart,
                             const uint8_t *buffer,
                             int            length)
{

    if (uart->len + length > FIFO_LENGTH) {
        abort();
    }
    memcpy(uart->buffer + uart->len, buffer, length);
    uart->len += length;
       DB_PRINT ("length: %d uart->len: %d\n", length, uart->len);
}

static int ath_apbuart_can_receive(void *opaque)
{
    UART *uart = ATH_APB_UART(opaque);
    return FIFO_LENGTH - uart->len;
}

static void ath_apbuart_receive(void *opaque, const uint8_t *buf, int size)
{
    UART *uart = ATH_APB_UART(opaque);
//printf("here i ammmmmmmmm\n");
   // if (uart->control & UART_RECEIVE_ENABLE) {

        uart_add_to_fifo(uart, buf, size);

//        uart->csr = UART_RX_CSR;

        if (uart->control & UART_RECEIVE_INTERRUPT) {
            qemu_irq_pulse(uart->irq);
        }
   // }
}

static uint64_t ath_apbuart_read(void *opaque, hwaddr addr,
                                   unsigned size)
{
    UART *uart = ATH_APB_UART(opaque);
    uint32_t val = 0;
    addr &= 0xff;

    /* Unit registers */
    switch (addr) {
    case UARTDATA:
        val = uart_pop(uart);
        qemu_chr_fe_accept_input(&uart->chr);
        return val;

    case UARTCS:
        /* Read Only */
        return val = uart->status;

    case UARTCLOCK:
        return val = uart->clk;

    case UARTINT:
        /* Not supported */
        return val = uart->intr;
    case UARTINTEN:
        /* Not supported */
        return val = uart->intren;

    default:
        DB_PRINT ("Bad register offset: 0x%08lx\n", addr);
        return 0;
    }
     DB_PRINT ("Register offset: 0x%08lx val: 0x%08x\n", (uint64_t) addr &0xffffffff, val);
}

static void ath_apbuart_write(void *opaque, hwaddr addr,
                                uint64_t val, unsigned size)
{
    UART *uart = ATH_APB_UART(opaque);
    unsigned char  c    = 0;

    addr &= 0xff;
 DB_PRINT ("Register offset: 0x%08lx val: 0x%08lx\n", (uint64_t) addr &0xffffffff, val);
    /* Unit registers */
    switch (addr) {
    case UARTDATA:
        /* Transmit when character device available and transmitter enabled */
        if (qemu_chr_fe_backend_connected(&uart->chr)/* &&
            (uart->csr & UART_TX_CSR)*/) {
            c = val & 0xFF;
            /* XXX this blocks entire thread. Rewrite to use
             * qemu_chr_fe_write and background I/O callbacks */
            qemu_chr_fe_write_all(&uart->chr, &c, 1);
            /* Generate interrupt */
            if (uart->control & UART_TRANSMIT_INTERRUPT) {
                qemu_irq_pulse(uart->irq);
            }
        }
        return;

    case UARTCS:
        /* Read Only */
        uart->status = val;
        return;

    case UARTCLOCK:
        uart->clk = val;
        return;

    case UARTINT:
        /* Not supported */
        uart->intr = val;
        return;
    case UARTINTEN:
        /* Not supported */
        uart->intren = val;
        return;
    default:
        DB_PRINT ("Bad register offset: 0x%08lx val: 0x%08lx\n", addr, val);
        break;
    }


}

static const MemoryRegionOps ath_apbuart_ops = {
    .write      = ath_apbuart_write,
    .read       = ath_apbuart_read,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void ath_apbuart_init(Object *obj)
{
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);
    UART *uart = ATH_APB_UART(obj);

    sysbus_init_irq(sbd, &uart->irq);

    memory_region_init_io(&uart->iomem, obj, &ath_apbuart_ops, uart,
                          "uart_hs", UART_REG_SIZE);

    sysbus_init_mmio(sbd, &uart->iomem);

}

static void ath_apbuart_realize(DeviceState *dev, Error **errp)
{
    UART *uart = ATH_APB_UART(dev);

    /* This UART has no flow control, so we do not need to register
     * an event handler to deal with CHR_EVENT_BREAK.
     */
    qemu_chr_fe_set_handlers(&uart->chr,
                             ath_apbuart_can_receive,
                             ath_apbuart_receive,
                             NULL,
                             NULL, uart, NULL, true);
}

static void ath_apbuart_reset(DeviceState *d)
{
    UART *uart = ATH_APB_UART(d);

    /* Transmitter FIFO and shift registers are always empty in QEMU */
    uart->status =  UART_TRANSMIT_FIFO_EMPTY | UART_TRANSMIT_SHIFT_EMPTY;
    /* Everything is off */
    uart->control = 0;
    /* Flush receive FIFO */
    uart->csr = 0;
    uart->len = 0;
    uart->current = 0;
}

static Property ath_apbuart_properties[] = {
    DEFINE_PROP_CHR("chrdev", UART, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void ath_apbuart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
//    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

//    k->init = ath_apbuart_init;
    dc->realize = ath_apbuart_realize;
    dc->reset = ath_apbuart_reset;
    dc->props = ath_apbuart_properties;
}

static const TypeInfo ath_apbuart_info = {
    .name          = TYPE_ATH_APB_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(UART),
    .instance_init = ath_apbuart_init,
    .class_init    = ath_apbuart_class_init,
};

static void ath_apbuart_register_types(void)
{
    type_register_static(&ath_apbuart_info);
}

type_init(ath_apbuart_register_types)
