/*
 * xtrx kernel ABI definitions header file
 * Copyright (c) 2017 Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 * For more information, please visit: http://xtrx.io
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#ifndef XTRX_DEFS_H
#define XTRX_DEFS_H

#define XTRX_MMAP_TX_BUF_OFF		2097152

#define XTRX_MMAP_STAT_OFF		4194304

#define XTRX_MMAP_RX_OFF		67108864
#define XTRX_MMAP_TX_OFF		134217728


#define XTRX_IOCTL_SPI_TRANSACT		0x123456



/* HW specific */
#define XTRX_MSI_COUNT             4


/* Kernel shared mmap */

#define XTRX_KERN_MMAP_1PPS_IRQS   0
#define XTRX_KERN_MMAP_TX_IRQS     1
#define XTRX_KERN_MMAP_RX_IRQS     2
#define XTRX_KERN_MMAP_CTRL_IRQS   3

#define XTRX_KERN_MMAP_GPS_RX_IRQS 5
#define XTRX_KERN_MMAP_GPS_TX_IRQS 6
#define XTRX_KERN_MMAP_SIM_RX_IRQS 7
#define XTRX_KERN_MMAP_SIM_TX_IRQS 8
#define XTRX_KERN_MMAP_I2C_IRQS    9


#endif
