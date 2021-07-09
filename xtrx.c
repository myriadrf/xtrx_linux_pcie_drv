/*
 * xtrx - kernel driver support for XTRX PCIe
 *
 * Copyright (C) 2014, 2016, 2017  Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307,
 * USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/device.h>
#include <linux/time.h>
#include <linux/pps_kernel.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <asm/page.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

#include "xtrx_defs.h"

#define localparam static const uint32_t
#include "xtrxll_regs.vh"

#define MAX_XTRX_DEVS 64

#define DRV_NAME		"xtrx"
#define PFX			DRV_NAME ": "

#define DEVICE_NAME		DRV_NAME
#define CLASS_NAME		DRV_NAME


#define XTRX_UART_LINE_GPS		0
#define XTRX_UART_LINE_SIM		1

#ifndef PCI_EXP_DEVCTL_READRQ_4096B
#define PCI_EXP_DEVCTL_READRQ_4096B 0x5000
#endif

/* Move out to extrnal CMD */
#define GET_HWID_COMPAT(x) ((x >> 8) & 0xff)
#define MAKE_I2C_CMD(RD, RDZSZ, WRSZ, DEVNO, DATA)  (\
	(((RD) & 1U) << 31) | \
	(((RDZSZ) & 7U) << 28) | \
	(((WRSZ) & 3U) << 26) | \
	(((DEVNO) & 3U) << 24) | \
	(((DATA) & 0xffffffu) << 0))
#define MAKE_LP8758XX_I2C_CMD(BUS, REG, IN) \
	MAKE_I2C_CMD(0, 0, 2, (BUS), (REG) | ((u32)(IN) << 8))
#define MAKE_LP8758LMS_I2C_CMD(REG, IN) \
	MAKE_LP8758XX_I2C_CMD(3, REG, IN)


/* Serial virtual devices */
struct xtrx_dev;

enum xtrx_uart_types {
	XTRX_UART_GPS,        /**< UART device for GPS NMEA */
	XTRX_UART_SIM,        /**< UART device for T0 smartcard */
	XTRX_UART_NUM,
};


#define BUFS      32
#define BUF_SIZE  32768

#define BUF_SIZE_MIN       8192
#define BUF_SIZE_MAX    4194304

#define UART_PORT_OPEN 1


struct xtrx_dmabuf_nfo {
	void* virt;
	dma_addr_t phys;
};

typedef enum xtrx_interrupt_type {
	XTRX_MSI_4,
	XTRX_MSI_SINGLE,
	XTRX_LEGACY
} xtrx_interrupt_type_t;


enum xtrx_pwr_blocks {
	XTRX_BLK_CHAR_DEV = 1,
	XTRX_BLK_GPS_UART = 2,
	XTRX_BLK_SIM_UART = 4,
};

struct xtrx_dev {
	struct xtrx_dev *next;   /* next device in list */
	unsigned devno;
	unsigned locked_msk;
	unsigned valid; /* usage counter */

	spinlock_t slock;

	struct cdev cdev;
	struct device* cdevice;

	xtrx_interrupt_type_t inttype;

	wait_queue_head_t queue_ctrl;  /* wait queue ctrl */
	wait_queue_head_t queue_i2c;   /* wait queue i2c */
	wait_queue_head_t queue_tx;    /* wait queue tx */
	wait_queue_head_t queue_rx;    /* wait queue rx */
	wait_queue_head_t onepps_ctrl;

	struct pci_dev *pdev;     /* pci device */

	struct pps_device *pps;   /* 1PPS events from GPS */

	void __iomem *bar0_addr;
	void __iomem *bar1_addr;

	void  *shared_mmap;

	unsigned buf_rx_size; /* actual size of each buffer in the RX ring */
	unsigned buf_tx_size; /* actual size of each buffer in the TX ring */

	struct xtrx_dmabuf_nfo buf_rx[BUFS];  /* RX bufs */
	struct xtrx_dmabuf_nfo buf_tx[BUFS];  /* TX bufs */

	struct uart_port port_gps;
	struct uart_port port_sim;


	unsigned gps_ctrl_state;
	unsigned sim_ctrl_state;
	u32 hwid;
	u32 pwr_msk;
};


static struct xtrx_dev *xtrx_list = NULL;
static int devices = 0;
static dev_t dev_first; /* Global variable for the first device number */
static struct class*  xtrx_class  = NULL;


MODULE_AUTHOR("Sergey Kostanbaev <sergey.kostanbaev@fairwaves.co>");
MODULE_DESCRIPTION("XTRX low-level PCIe driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");

/* Some ARM platform pci_alloc_consistent() reports VA that can't be
 * mmaped to userspce. Convertion DMA->PA->VA does the trick on that
 * platforms
 */
#ifdef CONFIG_CPU_RK3399
#define VA_DMA_ADDR_FIXUP
#endif


static void xtrx_writel(struct xtrx_dev *dev, unsigned int off, unsigned int value)
{
	iowrite32(cpu_to_be32(value), (void __iomem *)((unsigned long)dev->bar0_addr + 4*off));
}

static unsigned int xtrx_readl(struct xtrx_dev *dev, unsigned int off)
{
	return be32_to_cpu(ioread32((void __iomem *)((unsigned long)dev->bar0_addr + 4*off)));
}

static int xtrx_power_op(struct xtrx_dev *dev, int on, unsigned mask) 
{
	unsigned long flags;

	int do_pwr_op = 0;
	spin_lock_irqsave(&dev->slock, flags);
	if (on) {
		if (dev->pwr_msk == 0) {
			do_pwr_op = 1; 
		}
		dev->pwr_msk |= mask;
	} else {
		if (dev->pwr_msk) {
			dev->pwr_msk &= ~mask;
			if (dev->pwr_msk == 0) {
				do_pwr_op = 1;
			}
		}
	}
	spin_unlock_irqrestore(&dev->slock, flags);

	if (!do_pwr_op)
		return 0;

	printk(KERN_NOTICE PFX " 3V3 CTRL:%d\n", on);
	if (on) {
		xtrx_writel(dev, UL_GP_ADDR + GP_PORT_WR_TMP102,
				MAKE_LP8758LMS_I2C_CMD(0x0c, 0xfc));
		xtrx_writel(dev, UL_GP_ADDR + GP_PORT_WR_TMP102,
				MAKE_LP8758LMS_I2C_CMD(0x04, 0x88));
	} else {
		xtrx_writel(dev, UL_GP_ADDR + GP_PORT_WR_TMP102,
				MAKE_LP8758LMS_I2C_CMD(0x04, 0xc8));
	}
	return 1;
}

#define PORT_XTRX 0x03300330

/*
 * serial core request to check if uart tx fifo is empty
 */
static unsigned int xtrx_uart_tx_empty(struct uart_port *port)
{
	// TODO
	//printk(KERN_NOTICE PFX "Start TX empty?: %d\n", port->line);
	return TIOCSER_TEMT;
}

static struct xtrx_dev *xtrx_dev_from_uart_port(struct uart_port *port)
{
	if (port->line % XTRX_UART_NUM == XTRX_UART_LINE_SIM) {
		return (struct xtrx_dev *)((unsigned long)port - offsetof(struct xtrx_dev, port_sim));
	} else if (port->line % XTRX_UART_NUM == XTRX_UART_LINE_GPS) {
		return (struct xtrx_dev *)((unsigned long)port - offsetof(struct xtrx_dev, port_gps));
	} else {
		return NULL;
	}
}

static void xtrx_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct xtrx_dev *dev;
	if (port->line % XTRX_UART_NUM == XTRX_UART_LINE_SIM) {
		dev = xtrx_dev_from_uart_port(port);
		if (!(mctrl & TIOCM_RTS)) {
		    dev->sim_ctrl_state |= WR_SIM_CTRL_RESET;
		} else {
		    dev->sim_ctrl_state &= ~WR_SIM_CTRL_RESET;
		}
		xtrx_writel(dev, GP_PORT_WR_SIM_CTRL, dev->sim_ctrl_state);

		printk(KERN_NOTICE PFX "SIM mctrl=%x\n", dev->sim_ctrl_state);
	}
}

static unsigned int xtrx_uart_get_mctrl(struct uart_port *port)
{
	return TIOCM_CTS;
}

static void xtrx_uart_stop_tx(struct uart_port *port)
{
	// TODO
}

static void xtrx_uart_start_tx(struct uart_port *port)
{
	// TODO
	//printk(KERN_NOTICE PFX "Start TX: %d\n", port->line);
}

static void xtrx_uart_stop_rx(struct uart_port *port)
{
	// TODO
}

static void xtrx_uart_enable_ms(struct uart_port *port)
{
}

static void xtrx_uart_break_ctl(struct uart_port *port, int ctl)
{
}


static void xtrx_uart_do_rx(struct uart_port *port, unsigned* fifo_used);

static int xtrx_uart_startup(struct uart_port *port)
{
	unsigned long flags;
	struct xtrx_dev *dev = xtrx_dev_from_uart_port(port);
	if (port->line % XTRX_UART_NUM == XTRX_UART_LINE_SIM) {
		spin_lock_irqsave(&port->lock, flags);

		if (dev->sim_ctrl_state & WR_SIM_CTRL_ENABLE) {
		    spin_unlock_irqrestore(&port->lock, flags);
		    printk(KERN_NOTICE PFX "Port %d is already claimed!\n", port->line);
		    return -EBUSY;
		}

		dev->sim_ctrl_state = WR_SIM_CTRL_ENABLE;
		xtrx_writel(dev, GP_PORT_WR_SIM_CTRL, dev->sim_ctrl_state);

		spin_unlock_irqrestore(&port->lock, flags);

		xtrx_power_op(dev, 1, XTRX_BLK_SIM_UART);
	} else {
		dev->gps_ctrl_state = UART_PORT_OPEN;

		spin_lock(&dev->port_gps.lock);
		if (dev->gps_ctrl_state) {
			unsigned tx_fifo_used;
			xtrx_uart_do_rx(&dev->port_gps, &tx_fifo_used);
			//xtrx_uart_do_tx(&dev->port_gps, tx_fifo_used);
		}
		spin_unlock(&dev->port_gps.lock);

		xtrx_power_op(dev, 1, XTRX_BLK_GPS_UART);
	}

	printk(KERN_NOTICE PFX "Port opened: %d\n", port->line);
	return 0;
}

static void xtrx_uart_shutdown(struct uart_port *port)
{
	unsigned long flags;

	struct xtrx_dev *dev = xtrx_dev_from_uart_port(port);
	if (port->line % XTRX_UART_NUM == XTRX_UART_LINE_SIM) {
		spin_lock_irqsave(&port->lock, flags);

		dev->sim_ctrl_state = 0;
		xtrx_writel(dev, GP_PORT_WR_SIM_CTRL, dev->sim_ctrl_state);

		spin_unlock_irqrestore(&port->lock, flags);

		xtrx_power_op(dev, 0, XTRX_BLK_SIM_UART);
	} else {
		dev->gps_ctrl_state = 0;

		xtrx_power_op(dev, 0, XTRX_BLK_GPS_UART);
	}

	printk(KERN_NOTICE PFX "Port closed: %d\n", port->line);
}

static void xtrx_uart_set_termios(struct uart_port *port,
				 struct ktermios *new,
				 struct ktermios *old)
{
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	// TODO: Set HW registers
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *xtrx_uart_type(struct uart_port *port)
{
	return (port->type == PORT_XTRX) ? "xtrx_uart" : NULL;
}

static int xtrx_uart_request_port(struct uart_port *port)
{
	printk(KERN_NOTICE PFX "Port request: %d\n", port->line);
	return 0;
}

static void xtrx_uart_release_port(struct uart_port *port)
{
	printk(KERN_NOTICE PFX "Port release: %d\n", port->line);
}

static void xtrx_uart_config_port(struct uart_port *port, int flags)
{
	printk(KERN_NOTICE PFX "Port config: %d (%x)\n", port->line, flags);

	if (flags & UART_CONFIG_TYPE) {
		if (xtrx_uart_request_port(port))
			return;
		port->type = PORT_XTRX;
	}
}

static int xtrx_uart_verify_port(struct uart_port *port,
				 struct serial_struct *serinfo)
{
	if (port->type != PORT_XTRX)
		return -EINVAL;
	return 0;
}

void xtrx_uart_do_rx(struct uart_port *port, unsigned* fifo_used)
{
	struct tty_port *tty_port = &port->state->port;
	struct xtrx_dev *dev = xtrx_dev_from_uart_port(port);
	unsigned int max_count;
	char flag;
	int processed = 0;

	max_count = 33;
	do {
		unsigned int c;
		c = xtrx_readl(dev, (port->line % XTRX_UART_NUM == XTRX_UART_LINE_GPS) ?
					GP_PORT_RD_UART_RX : GP_PORT_RD_SIM_RX);

		if (fifo_used)
			*fifo_used = ((c >> UART_FIFOTX_USED_OFF) & ((1 << UART_FIFOTX_USED_BITS) - 1)) + ((c & (1 << UART_FIFOTX_EMPTY)) ? 0 : 1);

		if (c & (1 << UART_FIFORX_EMPTY))
			break;

		//printk(KERN_NOTICE PFX "Char %d: %x\n",  port->line, c);
		port->icount.rx++;
		flag = TTY_NORMAL;
		c &= 0xff;

		tty_insert_flip_char(tty_port, c, flag);

		processed = 1;
	} while (--max_count);

	if (processed) {
		spin_unlock(&port->lock);
		tty_flip_buffer_push(tty_port);
		spin_lock(&port->lock);
	}
}


static void xtrx_uart_do_tx(struct uart_port *port, unsigned fifo_used)
{
	struct circ_buf *xmit;
	unsigned int max_count;
	struct xtrx_dev *dev = xtrx_dev_from_uart_port(port);

	if (port->x_char) {
		xtrx_writel(dev, (port->line % XTRX_UART_NUM == XTRX_UART_LINE_GPS) ?
				    GP_PORT_WR_UART_TX : GP_PORT_WR_SIM_TX,
			    port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_tx_stopped(port)) {
		xtrx_uart_stop_tx(port);
		return;
	}

	xmit = &port->state->xmit;
	if (uart_circ_empty(xmit))
		goto txq_empty;

	max_count = port->fifosize - fifo_used;
	while (max_count--) {
		unsigned int c;

		c = xmit->buf[xmit->tail] & 0xff;
		//printk(KERN_NOTICE PFX "Char: %x\n", c);
		xtrx_writel(dev, (port->line % XTRX_UART_NUM == XTRX_UART_LINE_GPS) ?
				    GP_PORT_WR_UART_TX : GP_PORT_WR_SIM_TX, c);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit))
			break;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		goto txq_empty;
	return;

txq_empty:
	/* nothing to send, disable transmit interrupt */
	// TODO
	return;
}

/*
static int xtrx_uart_verify_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	printk(KERN_NOTICE PFX "UART IOCTL %d: %08x -> %016lx\n",
		port->line, cmd, arg);
	return -EINVAL;
}
*/

static struct uart_ops xtrx_uart_ops = {
	.tx_empty	= xtrx_uart_tx_empty,
	.get_mctrl	= xtrx_uart_get_mctrl,
	.set_mctrl	= xtrx_uart_set_mctrl,
	.start_tx	= xtrx_uart_start_tx,
	.stop_tx	= xtrx_uart_stop_tx,
	.stop_rx	= xtrx_uart_stop_rx,
	.enable_ms	= xtrx_uart_enable_ms,
	.break_ctl	= xtrx_uart_break_ctl,
	.startup	= xtrx_uart_startup,
	.shutdown	= xtrx_uart_shutdown,
	.set_termios	= xtrx_uart_set_termios,
	.type		= xtrx_uart_type,
	.release_port	= xtrx_uart_release_port,
	.request_port	= xtrx_uart_request_port,
	.config_port	= xtrx_uart_config_port,
	.verify_port	= xtrx_uart_verify_port,
	//.ioctl          = xtrx_uart_verify_ioctl,
};


// TODO get rid of constant
#define TTY_XTRX_MAJOR 234

static struct uart_driver xtrx_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "xtrxuart",
	.dev_name	= "ttyXTRX",
	.major		= TTY_XTRX_MAJOR,
	.minor		= 64,
	.nr		= XTRX_UART_NUM * MAX_XTRX_DEVS,
	.cons		= NULL,
};


// TTY functions
static int xtrx_uart_init(struct xtrx_dev* dev, unsigned xtrx_no)
{
	int ret;

	// Register GPS port
	dev->port_gps.membase = (void __iomem *)((unsigned long)dev->bar0_addr + GP_PORT_RD_UART_RX*4);
	dev->port_gps.iotype = UPIO_MEM;
	//dev->port_gps.irq = -1;
	dev->port_gps.ops = &xtrx_uart_ops;
	dev->port_gps.flags = UPF_BOOT_AUTOCONF;
	dev->port_gps.dev = &dev->pdev->dev;
	dev->port_gps.fifosize = 32;
	dev->port_gps.uartclk = 9600*16;
	dev->port_gps.line = xtrx_no * XTRX_UART_NUM + XTRX_UART_LINE_GPS;

	ret = uart_add_one_port(&xtrx_uart_driver, &dev->port_gps);
	if (ret) {
		printk(KERN_NOTICE PFX "Unable to initialize UART GPS port: %d\n", ret);
		return ret;
	}

	// Register SIM port
	dev->port_sim.membase = (void __iomem *)((unsigned long)dev->bar0_addr + GP_PORT_RD_SIM_RX*4);
	dev->port_sim.iotype = UPIO_MEM;
	//dev->port_sim.irq = -1;
	dev->port_sim.ops = &xtrx_uart_ops;
	dev->port_sim.flags = UPF_BOOT_AUTOCONF;
	dev->port_sim.dev = &dev->pdev->dev;
	dev->port_sim.fifosize = 32;
	dev->port_sim.uartclk = 9600*16;
	dev->port_sim.line = xtrx_no * XTRX_UART_NUM + XTRX_UART_LINE_SIM;

	ret = uart_add_one_port(&xtrx_uart_driver, &dev->port_sim);
	if (ret) {
		uart_remove_one_port(&xtrx_uart_driver, &dev->port_gps);
		printk(KERN_NOTICE PFX "Unable to initialize UART SIM port: %d\n", ret);
		return ret;
	}

	dev->sim_ctrl_state = 0;
	return 0;
}

static void xtrx_uart_deinit(struct xtrx_dev* dev)
{
	uart_remove_one_port(&xtrx_uart_driver, &dev->port_gps);
	uart_remove_one_port(&xtrx_uart_driver, &dev->port_sim);
}

// DMA functions
static void xtrx_set_dma_bufs(struct xtrx_dev *d, struct xtrx_dmabuf_nfo *pbufs, unsigned config_off, unsigned len)
{
	int i;
	unsigned len_qw = len / 16;

	//Initialize DMA buffers
	for (i = 0; i < BUFS; i++) {
		uintptr_t addr = ((uintptr_t)d->bar0_addr) + config_off + 4 * i;
		uint32_t reg   = ((len_qw - 1) & 0xFFF) | (0xFFFFF000 & pbufs[i].phys);
		printk(KERN_NOTICE PFX "buf[%d]=%lx [virt %p] => %08x\n", i, (unsigned long)pbufs[i].phys, pbufs[i].virt, reg);

		iowrite32(cpu_to_be32(reg), (void __iomem *)(addr));
		memset(pbufs[i].virt, i+1, len);
	}
}


static void xtrx_update_rxdma_len(struct xtrx_dev *d, struct xtrx_dmabuf_nfo *pbufs, unsigned len)
{
	if ((GET_HWID_COMPAT(d->hwid) >= 1)) {
		uint32_t pktsz = (unsigned)((len / 16) - 1) | (1U << 31);
		xtrx_writel(d, UL_RXDMA_ADDR + 32, pktsz);
	} else {
		xtrx_set_dma_bufs(d, pbufs, 0x800, len);
	}
}

static int xtrx_allocdma(struct xtrx_dev *d, struct xtrx_dmabuf_nfo *pbufs, unsigned config_off, unsigned buflen)
{
	int i;
	for (i = 0; i < BUFS; i++) {
		pbufs[i].virt = pci_alloc_consistent(d->pdev, buflen, &pbufs[i].phys);
		if (!pbufs[i].virt) {
			printk(KERN_INFO PFX "Failed to allocate %d DMA buffer", i);
			for (; i >= 0; --i) {
				pci_free_consistent(d->pdev, buflen, pbufs[i].virt, pbufs[i].phys);
			}
			return -1;
		}
	}

	xtrx_set_dma_bufs(d, pbufs, config_off, buflen);
	return 0;
}

static int xtrx_allocdma_tx(struct xtrx_dev *d, unsigned size)
{
	int res = xtrx_allocdma(d, d->buf_tx, 0xC00, size);
	if (res) {
		d->buf_tx_size = 0;
		return res;
	}

	d->buf_tx_size = size;
	return 0;
}

static int xtrx_allocdma_rx(struct xtrx_dev *d, unsigned size)
{
	int res = xtrx_allocdma(d, d->buf_rx, 0x800, size);
	if (res) {
		d->buf_rx_size = 0;
		return res;
	}

	xtrx_update_rxdma_len(d, d->buf_rx, size);
	d->buf_rx_size = size;
	return 0;
}



static void xtrx_freedma_rx(struct xtrx_dev *d)
{
	int i;
	for (i = 0; i < BUFS; i++) {
		pci_free_consistent(d->pdev, d->buf_rx_size, d->buf_rx[i].virt, d->buf_rx[i].phys);
	}
	d->buf_rx_size = 0;
}

static void xtrx_freedma_tx(struct xtrx_dev *d)
{
	int i;
	for (i = 0; i < BUFS; i++) {
		pci_free_consistent(d->pdev, d->buf_tx_size, d->buf_tx[i].virt, d->buf_tx[i].phys);
	}
	d->buf_tx_size = 0;
}


// Interrupt routine functions
static void xtrx_interrupt_gpspps(struct xtrx_dev *xtrxdev)
{
	struct pps_event_time ts;
	pps_get_ts(&ts);
	pps_event(xtrxdev->pps, &ts, PPS_CAPTUREASSERT, NULL);
}

static void xtrx_process_l_interrupts(struct xtrx_dev *xtrxdev, uint32_t imask)
{
	if (imask & (1 << INT_RFIC0_SPI)) {
		atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_CTRL_IRQS);
		wake_up_interruptible(&xtrxdev->queue_ctrl);
	}

	if (imask & (1 << INT_I2C)) {
		atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_I2C_IRQS);
		wake_up_interruptible(&xtrxdev->queue_i2c);
	}

	if (imask & (1 << INT_GPS_UART_RX)) {
		// TODO tty or user notificatio
		//atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_GPS_RX_IRQS);
		//wake_up_interruptible(&xtrxdev->uart_gps_rx);

		spin_lock(&xtrxdev->port_gps.lock);
		if (xtrxdev->gps_ctrl_state) {
			unsigned tx_fifo_used;
			xtrx_uart_do_rx(&xtrxdev->port_gps, &tx_fifo_used);
			xtrx_uart_do_tx(&xtrxdev->port_gps, tx_fifo_used);
		}
		spin_unlock(&xtrxdev->port_gps.lock);
	}

	if (imask & (1 << INT_SIM_UART_RX)) {
		// TODO tty or user notificatio
		//atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_SIM_RX_IRQS);
		//wake_up_interruptible(&xtrxdev->uart_gps_rx);

		spin_lock(&xtrxdev->port_sim.lock);
		if (xtrxdev->sim_ctrl_state & WR_SIM_CTRL_ENABLE) {
			unsigned tx_fifo_used;
			xtrx_uart_do_rx(&xtrxdev->port_sim, &tx_fifo_used);
			xtrx_uart_do_tx(&xtrxdev->port_sim, tx_fifo_used);
		}
		spin_unlock(&xtrxdev->port_sim.lock);
	}
}

static irqreturn_t xtrx_msi_irq_pps(int irq, void *data)
{
	struct xtrx_dev *xtrxdev = data;
	xtrx_interrupt_gpspps(xtrxdev);

	atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_1PPS_IRQS);
	wake_up_interruptible(&xtrxdev->onepps_ctrl);
	return IRQ_HANDLED;
}

static irqreturn_t xtrx_msi_irq_tx(int irq, void *data)
{
	struct xtrx_dev *xtrxdev = data;
	atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_TX_IRQS);
	wake_up_interruptible(&xtrxdev->queue_tx);
	return IRQ_HANDLED;
}

static irqreturn_t xtrx_msi_irq_rx(int irq, void *data)
{
	struct xtrx_dev *xtrxdev = data;
	atomic_inc((atomic_t*)xtrxdev->shared_mmap + XTRX_KERN_MMAP_RX_IRQS);
	wake_up_interruptible(&xtrxdev->queue_rx);
	return IRQ_HANDLED;
}

static irqreturn_t xtrx_msi_irq_other(int irq, void *data)
{
	struct xtrx_dev *xtrxdev = data;
	uint32_t imask;

	imask = xtrx_readl(xtrxdev, GP_PORT_RD_INTERRUPTS);
	xtrx_process_l_interrupts(xtrxdev, imask);

	return IRQ_HANDLED;
}


static irqreturn_t xtrx_irq_legacy(int irq, void *data)
{
	struct xtrx_dev *xtrxdev = data;
	uint32_t imask = xtrx_readl(xtrxdev, GP_PORT_RD_INTERRUPTS);
	if (imask == 0) {
		return IRQ_NONE;
	}

	if (imask & (1 << (INT_1PPS))) {
		xtrx_msi_irq_pps(irq, data);
	}
	if (imask & (1 << (INT_DMA_TX))) {
		xtrx_msi_irq_tx(irq, data);
	}
	if (imask & (1 << (INT_DMA_RX))) {
		xtrx_msi_irq_rx(irq, data);
	}
	xtrx_process_l_interrupts(xtrxdev, imask);

	return IRQ_HANDLED;
}

static irqreturn_t xtrx_msi_irq_single(int irq, void *data)
{
	xtrx_irq_legacy(irq, data);
	return IRQ_HANDLED;
}


static struct pps_source_info xtrx_pps_info = {
	.name		= "xtrx_pps",
	.path		= "",
	.mode		= PPS_CAPTUREASSERT|PPS_OFFSETASSERT|PPS_ECHOASSERT|
			  PPS_CANWAIT|PPS_TSFMT_TSPEC,
	.owner 		= THIS_MODULE,
};

static int xtrxfd_open(struct inode *inode, struct file *filp)
{
	struct xtrx_dev *dev;
	unsigned long flags;
	int granted = 0;

	dev = container_of(inode->i_cdev, struct xtrx_dev, cdev);
	filp->private_data = dev;

	spin_lock_irqsave(&dev->slock, flags);
	if ((dev->locked_msk & XTRX_BLK_CHAR_DEV) == 0) {

		dev->locked_msk |= XTRX_BLK_CHAR_DEV;
		granted = 1;
	}
	spin_unlock_irqrestore(&dev->slock, flags);

	return (granted) ? 0 : -EBUSY;
}

static int xtrxfd_release(struct inode *inode, struct file *filp)
{
	struct xtrx_dev *xtrxdev = filp->private_data;
	unsigned long flags;

	if (xtrxdev->valid == 0) {
		printk(KERN_INFO PFX "XTRX:%d dev is invalid!\n", xtrxdev->devno);
		return 0;
	}

	xtrx_power_op(xtrxdev, 0, XTRX_BLK_CHAR_DEV);

	spin_lock_irqsave(&xtrxdev->slock, flags);
	xtrxdev->locked_msk &= ~XTRX_BLK_CHAR_DEV;
	spin_unlock_irqrestore(&xtrxdev->slock, flags);
	return 0;
}

static ssize_t xtrxfd_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *f_pos)
{
	struct xtrx_dev *xtrxdev = filp->private_data;
	int i;
	int timeout = 2 * HZ;

	if (!xtrxdev)
		return -ENODEV;

	if (!f_pos)
		return -EINVAL;

	if (count > 1) {
		timeout = HZ * count / 1000;
		if (timeout < 1)
			timeout = 1;
	}

	switch (*f_pos) {
	case XTRX_KERN_MMAP_1PPS_IRQS:
		i = wait_event_interruptible_timeout(
			xtrxdev->onepps_ctrl,
			atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_1PPS_IRQS) != 0,
			2 * HZ);

		if (!i) {
			return -EAGAIN;
		}
		return atomic_xchg(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_1PPS_IRQS, 0);

	case XTRX_KERN_MMAP_CTRL_IRQS:

		i = wait_event_interruptible_timeout(
			xtrxdev->queue_ctrl,
			atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_CTRL_IRQS) != 0,
			2 * HZ);

		if (!i) {
			return -EAGAIN;
		}
		return atomic_xchg(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_CTRL_IRQS, 0);

	case XTRX_KERN_MMAP_I2C_IRQS:
		i = wait_event_interruptible_timeout(
			xtrxdev->queue_i2c,
			atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_I2C_IRQS) != 0,
			2 * HZ);

		if (!i) {
			return -EAGAIN;
		}
		return atomic_xchg(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_I2C_IRQS, 0);

	case XTRX_KERN_MMAP_RX_IRQS:
		i = wait_event_interruptible_timeout(
			xtrxdev->queue_rx,
			atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_RX_IRQS) != 0,
			timeout/*2 * HZ*/);

		if (!i) {
			return -EAGAIN;
		}
		return atomic_xchg(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_RX_IRQS, 0);

	case XTRX_KERN_MMAP_TX_IRQS:
		i = wait_event_interruptible_timeout(
			xtrxdev->queue_tx,
			atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_TX_IRQS) != 0,
			timeout/*2 * HZ*/);

		if (!i) {
			return -EAGAIN;
		}
		return atomic_xchg(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_TX_IRQS, 0);
	}


	return -EINVAL;
}

static ssize_t xtrxfd_write(struct file *filp, const char __user *buf, size_t count,
			     loff_t *f_pos)
{
	return -EINVAL;
}

static unsigned int xtrxfd_poll(struct file *filp, poll_table *wait)
{
	struct xtrx_dev *xtrxdev = filp->private_data;
	unsigned int mask = 0;

	poll_wait(filp, &xtrxdev->queue_ctrl, wait);
	poll_wait(filp, &xtrxdev->queue_tx, wait);
	poll_wait(filp, &xtrxdev->queue_rx, wait);

	if (atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_CTRL_IRQS))
		mask |= POLLIN | POLLRDNORM;

	if (atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_RX_IRQS))
		mask |= POLLIN | POLLRDBAND;

	if (atomic_read(((atomic_t*)xtrxdev->shared_mmap) + XTRX_KERN_MMAP_TX_IRQS))
		mask |= POLLOUT | POLLWRBAND;

	return mask;
}

// TODO more ioctls
// FIXME fix ioctl numbers
static long xtrxfd_ioctl(struct file *filp,
			 unsigned int ioctl_num,/* The number of the ioctl */
			 unsigned long ioctl_param) /* The parameter to it */
{
	int err;
	struct xtrx_dev *xtrxdev = filp->private_data;
	printk(KERN_WARNING PFX "ioctl(%x, %lx) [%lx]", ioctl_num, ioctl_param, (unsigned long)xtrxdev->bar0_addr);

	switch (ioctl_num) {
		case 0x123458: {
			int i;
			for ( i = 0; (i < BUFS); ++i) {
				memset( xtrxdev->buf_rx[i].virt, i + 1, xtrxdev->buf_rx_size);
			}
			return 0;
		}
		case 0x123459: {
			if (ioctl_param > xtrxdev->buf_rx_size)
				return -E2BIG;
			if (ioctl_param < BUF_SIZE_MIN)
				return -EINVAL;

		    xtrx_update_rxdma_len(xtrxdev, xtrxdev->buf_rx, ioctl_param);
		    return 0;
		}
		case 0x12345A: {
			if (ioctl_param > BUF_SIZE_MAX || ioctl_param < BUF_SIZE_MIN) {
				printk(KERN_WARNING PFX " INCORRECT SZ!\n");
				return -EINVAL;
			}
			if (ioctl_param <= xtrxdev->buf_rx_size)
				return xtrxdev->buf_rx_size;

			xtrx_freedma_rx(xtrxdev);
			err = xtrx_allocdma_rx(xtrxdev, ioctl_param);
			if (err < 0) {
				return err;
			}
			return xtrxdev->buf_rx_size;
		}
		case 0x12345B: {
			return xtrx_power_op(xtrxdev, ioctl_param, XTRX_BLK_CHAR_DEV);
		}
		default: return -EINVAL;
	}
	return 0;
}

static void xtrxfd_vma_open(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE PFX "VMA open, virt %lx, phys %lx\n",
		vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
}

static void xtrxfd_vma_close(struct vm_area_struct *vma)
{
	printk(KERN_NOTICE PFX "VMA close.\n");
}

static struct vm_operations_struct xtrxfd_remap_vm_ops = {
	.open =  xtrxfd_vma_open,
	.close = xtrxfd_vma_close,
};

static int xtrxfd_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct xtrx_dev *xtrxdev = filp->private_data;
	enum xtrx_mmap_region {
		REGION_STAT,
		REGION_CTRL,
		REGION_TX_DEVMEM,
		REGION_RX_BUF,
		REGION_TX_BUF,
	} region;

	if (vma->vm_pgoff == (XTRX_MMAP_STAT_OFF >> PAGE_SHIFT)) {
		region = REGION_STAT;
	} else if (vma->vm_pgoff == 0) {
		region = REGION_CTRL;
	} else if (vma->vm_pgoff == (XTRX_MMAP_TX_BUF_OFF >> PAGE_SHIFT)) {
		region = REGION_TX_DEVMEM;
	} else if (vma->vm_pgoff == (XTRX_MMAP_RX_OFF >> PAGE_SHIFT)) {
		region = REGION_RX_BUF;
	} else if (vma->vm_pgoff == (XTRX_MMAP_TX_OFF >> PAGE_SHIFT)) {
		region = REGION_TX_BUF;
	} else {
		printk(KERN_NOTICE PFX "call: UNKNOWN REGION: VMA=%p vma->vm_pgoff=%lu\n", vma, vma->vm_pgoff);
		return -ENXIO;
	}

	printk(KERN_NOTICE PFX "call: REGION=%d VMA=%p vma->vm_pgoff=%lu\n",
		   region, vma, vma->vm_pgoff);

	if (region == REGION_STAT) {
		if ((vma->vm_end - vma->vm_start) != (1 << PAGE_SHIFT)) {
			return -EINVAL;
		}
		//vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		vma->vm_flags |= VM_LOCKED;

		if (remap_pfn_range(vma, vma->vm_start,
							virt_to_phys((void*)((unsigned long)xtrxdev->shared_mmap)) >> PAGE_SHIFT,
							vma->vm_end - vma->vm_start,
							vma->vm_page_prot))
			return -EAGAIN;

		vma->vm_ops = &xtrxfd_remap_vm_ops;
		xtrxfd_vma_open(vma);
		return 0;
	} else if (region == REGION_CTRL || region == REGION_TX_DEVMEM) {
		unsigned long pfn;
		int bar = (region == REGION_CTRL) ? 0 : 1;
		vma->vm_page_prot = pgprot_device(vma->vm_page_prot);
		vma->vm_flags |= VM_IO;
		pfn = pci_resource_start(xtrxdev->pdev, bar) >> PAGE_SHIFT;

		if (io_remap_pfn_range(vma, vma->vm_start, pfn,
					vma->vm_end - vma->vm_start,
					vma->vm_page_prot))
			return -EAGAIN;

		vma->vm_ops = &xtrxfd_remap_vm_ops;
		xtrxfd_vma_open(vma);
		return 0;
	} else if (region == REGION_RX_BUF || region == REGION_TX_BUF) {
		unsigned long pfn, off;
		unsigned i;
		int ret;
		struct xtrx_dmabuf_nfo *pbufs = (region == REGION_RX_BUF) ? xtrxdev->buf_rx : xtrxdev->buf_tx;
		unsigned bufsize = (region == REGION_RX_BUF) ? xtrxdev->buf_rx_size : xtrxdev->buf_tx_size;

		if ((vma->vm_end - vma->vm_start) != BUFS * bufsize) {
			printk(KERN_NOTICE PFX "mmap() :  end-start=%lx -> %lx\n",
				(long)(vma->vm_end - vma->vm_start), (long)(BUFS * bufsize));
			return -EINVAL;
		}

		//vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
		vma->vm_flags |= VM_LOCKED;

		for (i = 0, off = 0; i < BUFS; ++i, off += bufsize) {
#ifdef VA_DMA_ADDR_FIXUP
			void *va = phys_to_virt(dma_to_phys(&xtrxdev->pdev->dev, pbufs[i].phys));
#else
			void *va = pbufs[i].virt;
#endif
			pfn = page_to_pfn(virt_to_page(va));

			ret = remap_pfn_range(vma, vma->vm_start + off,
						pfn,
						bufsize,
						vma->vm_page_prot);
		}

		return ret;
	}

	return -EINVAL;
}

struct file_operations xtrx_fops = {
	.owner =   THIS_MODULE,
//	.llseek =  xtrxfd_llseek,
	.read =    xtrxfd_read,
	.write =   xtrxfd_write,
	.unlocked_ioctl =  xtrxfd_ioctl,
	.open =    xtrxfd_open,
	.poll =    xtrxfd_poll,
	.mmap =    xtrxfd_mmap,
	.release = xtrxfd_release,
};

static int xtrx_setup_cdev(struct xtrx_dev *xtrxdev)
{
	dev_t dev_num = dev_first + xtrxdev->devno;

	cdev_init(&xtrxdev->cdev, &xtrx_fops);
	xtrxdev->cdev.owner = THIS_MODULE;
	xtrxdev->cdev.ops = &xtrx_fops;
	return cdev_add (&xtrxdev->cdev, dev_num, 1);
}

static int xtrx_probe(struct pci_dev *pdev,
			const struct pci_device_id *id)
{
	struct xtrx_dev* xtrxdev;
	int err;
	void __iomem* bar0_addr;
	void __iomem* bar1_addr;
	unsigned xtrx_no = devices;

	printk(KERN_INFO PFX "Initializing %s\n", pci_name(pdev));
#ifdef VA_DMA_ADDR_FIXUP
	printk(KERN_INFO PFX "Buggy va/dma mapping on the platform! Fixup enabled\n");
#endif

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Cannot enable PCI device, "
			"aborting.\n");
		return err;
	}

	/* Reconfigure MaxReadReq to 4KB */
	pcie_capability_clear_and_set_word(pdev, PCI_EXP_DEVCTL,
					   PCI_EXP_DEVCTL_READRQ, PCI_EXP_DEVCTL_READRQ_4096B);

	pci_set_master(pdev);

	if (pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32))) {
		dev_err(&pdev->dev,"No suitable consistent DMA available.\n");
		goto err_disable_pdev;
	}

	/*
	 * Check for BARs.  We expect 0: 4KB
	 */
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM) ||
		pci_resource_len(pdev, 0) < 1 << 12) {
		dev_err(&pdev->dev, "Missing UL BAR, aborting.\n");
		err = -ENODEV;
		goto err_disable_pdev;
	}

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(&pdev->dev, "Cannot obtain PCI resources, "
			"aborting.\n");
		goto err_disable_pdev;
	}

	bar0_addr = pci_iomap(pdev, 0,  1 << 12);
	if (!bar0_addr) {
		dev_err(&pdev->dev, "Failed to map BAR 0.\n");
		goto err_free_res;
	}

	bar1_addr = pci_iomap(pdev, 1,  1 << 16);
	if (!bar0_addr) {
		dev_err(&pdev->dev, "Failed to map BAR 0.\n");
		goto err_unmap0;
	}

	xtrxdev = kzalloc(sizeof(*xtrxdev), GFP_KERNEL);
	if (!xtrxdev) {
		dev_err(&pdev->dev, "Failed to allocate memory.\n");
		err = -ENOMEM;
		goto err_unmap1;
	}
	pci_set_drvdata(pdev, xtrxdev);
	xtrxdev->bar0_addr = bar0_addr;
	xtrxdev->bar1_addr = bar1_addr;
	xtrxdev->devno = xtrx_no;
	xtrxdev->pdev = pdev;
	xtrxdev->locked_msk = 0;
	xtrxdev->gps_ctrl_state = 0;
	xtrxdev->sim_ctrl_state = 0;
	xtrxdev->pwr_msk = 0;
	xtrxdev->valid = 0;
	spin_lock_init(&xtrxdev->slock);

	xtrxdev->shared_mmap = (char*)get_zeroed_page(GFP_KERNEL);
	if (!xtrxdev->shared_mmap) {
		dev_err(&pdev->dev, "Failed to allocate kernel mmap memory.\n");
		err = -ENOMEM;
		goto err_alloc0;
	}

	xtrxdev->hwid = xtrx_readl(xtrxdev, GP_PORT_RD_HWCFG);

	xtrxdev->pps = pps_register_source(&xtrx_pps_info,
					PPS_CAPTUREASSERT | PPS_OFFSETASSERT);
	if (!xtrxdev->pps) {
		dev_err(&pdev->dev, "Failed to register 1PPS source.\n");
	}

	init_waitqueue_head(&xtrxdev->queue_ctrl);
	init_waitqueue_head(&xtrxdev->queue_tx);
	init_waitqueue_head(&xtrxdev->queue_rx);
	init_waitqueue_head(&xtrxdev->onepps_ctrl);
	init_waitqueue_head(&xtrxdev->queue_i2c);


#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,11,0)
	err = pci_enable_msi_range(pdev, XTRX_MSI_COUNT, XTRX_MSI_COUNT);
#else
	err = pci_alloc_irq_vectors(pdev, XTRX_MSI_COUNT, XTRX_MSI_COUNT, PCI_IRQ_MSI);
#endif
	if (err == XTRX_MSI_COUNT) {
		xtrxdev->inttype = XTRX_MSI_4;

		err = request_irq(pdev->irq + INT_1PPS, xtrx_msi_irq_pps, 0, "xtrx_pps", xtrxdev);
		if (err) {
			dev_err(&pdev->dev, "Failed to register CTRL MSI.\n");
			err = -ENODEV;
			goto err_msi;
		}

		err = request_irq(pdev->irq + INT_DMA_TX, xtrx_msi_irq_tx, 0, "xtrx_tx", xtrxdev);
		if (err) {
			dev_err(&pdev->dev, "Failed to register TX MSI.\n");
			err = -ENODEV;
			goto err_irq_0;
		}

		err = request_irq(pdev->irq + INT_DMA_RX, xtrx_msi_irq_rx, 0, "xtrx_rx", xtrxdev);
		if (err) {
			dev_err(&pdev->dev, "Failed to register RX MSI.\n");
			err = -ENODEV;
			goto err_irq_1;
		}

		err = request_irq(pdev->irq + 3, xtrx_msi_irq_other, 0, "xtrx_other", xtrxdev);
		if (err) {
			dev_err(&pdev->dev, "Failed to register OTHER MSI.\n");
			err = -ENODEV;
			goto err_irq_2;
		}
	} else {

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4,11,0)
		err = pci_enable_msi_range(pdev, 1, 1);
#else
		err = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_MSI);
#endif
		if (err == 1) {
			xtrxdev->inttype = XTRX_MSI_SINGLE;

			err = request_irq(pdev->irq, xtrx_msi_irq_single, 0, "xtrx_msi_single", xtrxdev);
			if (err) {
				dev_err(&pdev->dev, "Failed to register SINGLE MSI.\n");
				err = -ENODEV;
				goto err_msi;
			}	
		} else {
			dev_err(&pdev->dev, "Failed to enable MSI, falling back to legacy mode.\n");
			xtrxdev->inttype = XTRX_LEGACY;

			err = request_irq(pdev->irq, xtrx_irq_legacy, IRQF_SHARED, "xtrx_legacy", xtrxdev);
			if (err) {
				dev_err(&pdev->dev, "Failed to register Legacy interrupt.\n");
				err = -ENODEV;
				goto err_alloc1;
			}
		}
	}

	/* Clear pending interrupts */
	xtrx_readl(xtrxdev, GP_PORT_RD_INTERRUPTS);

	/* Enable interrupts */
	xtrx_writel(xtrxdev, GP_PORT_WR_INT_PCIE, (1U << INT_PCIE_I_FLAG) | 0xfff);

	// Mark that we don't have preallocated buffers
	xtrxdev->buf_rx_size = 0;
	xtrxdev->buf_tx_size = 0;

	memset(xtrxdev->buf_rx, 0, sizeof(xtrxdev->buf_rx));
	memset(xtrxdev->buf_tx, 0, sizeof(xtrxdev->buf_tx));

	err = xtrx_allocdma_rx(xtrxdev, BUF_SIZE);
	if (err) {
		dev_err(&pdev->dev, "Failed to register RX DMA buffers.\n");
		err = -ENODEV;
		goto err_irq_3;
	}

	err = xtrx_allocdma_tx(xtrxdev, BUF_SIZE);
	if (err) {
		dev_err(&pdev->dev, "Failed to register TX DMA buffers.\n");
		err = -ENODEV;
		goto err_rx_bufs;
	}

	err = xtrx_uart_init(xtrxdev, xtrx_no);
	if (err) {
		goto err_uart;
	}

	xtrxdev->valid = 1;
	xtrxdev->cdevice = device_create(xtrx_class,
					 &pdev->dev,
					 MKDEV(MAJOR(dev_first), MINOR(dev_first) + devices),
					 NULL,
					 DEVICE_NAME "%d",
					 devices);
	if (IS_ERR(xtrxdev->cdevice)) {
		printk(KERN_NOTICE PFX "Unable to register device class\n");
		goto failed_device;
	}

	err = xtrx_setup_cdev(xtrxdev);
	if (err) {
		printk(KERN_NOTICE PFX "Error %d initializing cdev\n", err);
		goto failed_cdev;
	}

	devices++;
	xtrxdev->next = xtrx_list;
	xtrx_list = xtrxdev;
	return 0;

	//cdev_del(&xtrxdev->cdev);
failed_cdev:
	device_destroy(xtrx_class, MKDEV(MAJOR(dev_first), MINOR(dev_first) + devices));
failed_device:
	xtrx_uart_deinit(xtrxdev);
err_uart:
	xtrx_freedma_tx(xtrxdev);
err_rx_bufs:
	xtrx_freedma_rx(xtrxdev);
err_irq_3:
	xtrx_writel(xtrxdev, GP_PORT_WR_INT_PCIE, (1U << INT_PCIE_I_FLAG));
	xtrx_readl(xtrxdev, GP_PORT_RD_INTERRUPTS);

	if (xtrxdev->inttype == XTRX_LEGACY) {
		free_irq(pdev->irq, xtrxdev);
	} else if (xtrxdev->inttype == XTRX_MSI_SINGLE) {
		free_irq(pdev->irq, xtrxdev);

		pci_disable_msi(pdev);
	} else if (xtrxdev->inttype == XTRX_MSI_4) {
		free_irq(pdev->irq + 3, xtrxdev);
err_irq_2:
		free_irq(pdev->irq + 2, xtrxdev);
err_irq_1:
		free_irq(pdev->irq + 1, xtrxdev);
err_irq_0:
		free_irq(pdev->irq + 0, xtrxdev);
err_msi:
		pci_disable_msi(pdev);
	}
err_alloc1:
	if (xtrxdev->pps) {
		pps_unregister_source(xtrxdev->pps);
	}
	free_page((unsigned long)xtrxdev->shared_mmap);
err_alloc0:
	kfree(xtrxdev);
err_unmap1:
	pci_iounmap(pdev, bar1_addr);
err_unmap0:
	pci_iounmap(pdev, bar0_addr);
err_free_res:
	pci_release_regions(pdev);

err_disable_pdev:
	pci_clear_master(pdev); /* Nobody seems to do this */

	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);
	return err;
};


static void xtrx_remove(struct pci_dev *pdev)
{
	struct xtrx_dev* xtrxdev = pci_get_drvdata(pdev);
	printk(KERN_INFO PFX "Removing device %s\n", pci_name(pdev));

	/* Disable interrupts */
	xtrx_writel(xtrxdev, GP_PORT_WR_INT_PCIE, (1U << INT_PCIE_I_FLAG));
	/* Clear pending interrupts */
	xtrx_readl(xtrxdev, GP_PORT_RD_INTERRUPTS);

	cdev_del(&xtrxdev->cdev);
	device_destroy(xtrx_class, MKDEV(MAJOR(dev_first), MINOR(xtrxdev->devno)));

	xtrx_uart_deinit(xtrxdev);

	if (xtrxdev->inttype == XTRX_LEGACY) {
		free_irq(pdev->irq, xtrxdev);
	} else if (xtrxdev->inttype == XTRX_MSI_SINGLE) {
		free_irq(pdev->irq, xtrxdev);

		pci_disable_msi(pdev);
	} else if (xtrxdev->inttype == XTRX_MSI_4) {
		free_irq(pdev->irq + 0, xtrxdev);
		free_irq(pdev->irq + 1, xtrxdev);
		free_irq(pdev->irq + 2, xtrxdev);
		free_irq(pdev->irq + 3, xtrxdev);

		pci_disable_msi(pdev);
	}
	if (xtrxdev->pps) {
		pps_unregister_source(xtrxdev->pps);
	}

	xtrxdev->valid = 0;

	pci_iounmap(pdev, xtrxdev->bar1_addr);
	pci_iounmap(pdev, xtrxdev->bar0_addr);
	pci_release_regions(pdev);

	pci_clear_master(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);

	devices--;
	// TODO: Unchain from list and free the memory

	xtrx_freedma_tx(xtrxdev);
	xtrx_freedma_rx(xtrxdev);

}

static struct pci_device_id xtrx_pci_table[] = {
	{ PCI_DEVICE(0x10EE, 0xFAE3),
	  .driver_data = 0 },
	{ PCI_DEVICE(0x10EE, 0x7011),
	  .driver_data = 1 },
	{ PCI_DEVICE(0x10EE, 0x7012),
	  .driver_data = 2 },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, xtrx_pci_table);

static struct pci_driver xtrx_driver = {
	.name		= DRV_NAME,
	.id_table	= xtrx_pci_table,
	.probe		= xtrx_probe,
	.remove		= xtrx_remove
};


static int __init xtrx_init(void)
{
	int err;

	err = uart_register_driver(&xtrx_uart_driver);
	if (err) {
		printk(KERN_NOTICE PFX "Unable to initialize UART driver: %d\n", err);
		goto failed_uart;
	}

	err = alloc_chrdev_region(&dev_first, 0, MAX_XTRX_DEVS, DRV_NAME);
	if (err) {
		printk(KERN_NOTICE PFX "Unable to allocate chrdev region: %d\n", err);
		goto failed_chrdev;
	}

	xtrx_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(xtrx_class)) {
		printk(KERN_NOTICE PFX "Unable to register xtrx class\n");
		goto failed_setup_cdev;
	}

	err = pci_register_driver(&xtrx_driver);
	if (err) {
		printk(KERN_NOTICE PFX "Unable to register PCI driver: %d\n", err);
		goto failed_pci;
	}
	return 0;

failed_pci:
	class_destroy(xtrx_class);
failed_setup_cdev:
	unregister_chrdev_region(dev_first, devices);
failed_chrdev:
	uart_unregister_driver(&xtrx_uart_driver);
failed_uart:
	return err;
}

static void __exit xtrx_cleanup(void)
{

	struct xtrx_dev *ptr = xtrx_list, *next;

	pci_unregister_driver(&xtrx_driver);

	class_destroy(xtrx_class);

	unregister_chrdev_region(dev_first, devices);

	uart_unregister_driver(&xtrx_uart_driver);


	while (ptr != NULL) {
		next = ptr->next;
		free_page((unsigned long)ptr->shared_mmap);
		kfree(ptr);
		ptr = next;
	}

}


module_init(xtrx_init);
module_exit(xtrx_cleanup);

