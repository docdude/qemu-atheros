/*
 * QEMU Atheros Components
 *
 * Copyright (c) 2018-2019 
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

#ifndef ATH_APBUART_H
#define ATH_APBUART_H

#include "hw/qdev.h"
#include "hw/sysbus.h"

/* APB UART */

static inline
DeviceState *ath_apbuart_create(hwaddr base,
                                  Chardev *serial,
                                  qemu_irq irq)
{
    DeviceState *dev;

    dev = qdev_create(NULL, "ath,apbuart");
    qdev_prop_set_chr(dev, "chrdev", serial);

    qdev_init_nofail(dev);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 0, base);

    sysbus_connect_irq(SYS_BUS_DEVICE(dev), 0, irq);

    return dev;
}
#endif /* ATH_APBUART_H */
