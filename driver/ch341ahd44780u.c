/*
 * Original source code used as basis for this kernel driver (p_displayis.c):
 *
 **********************************************************************
 ***********    Copyright (C) WCH 2013.11.28    ***************
 ***********        web: www.wch.cn     ***************
 ***********    AUTHOR: TECH33 (tech@wch.cn)    ***************
 ***********   Used for USB Interface Chip (CH341)  ***************
 ***********  Nanjing QinHeng Electronics Co.,Ltd ***************
 **********************************************************************
 *
 * Running Environment: Linux
 * This file is used for CH34x in Epp/MEM/I2C/SPI
 *
 **********************************************************************
 *
 * @brief ch341a USB bridge controller to Hitachi HD44780U 4x20
 *        characters display module, as sold e.g. by:
 *        https://www.electronic-software-shop.com/hardware/displays-usb/
 *        Linux kernel driver
 *
 * @author Ingo A. Kubbilun (ingo.kubbilun@gmail.com), 2023/03/12
 *
 * a) the USB bridge controller is operated in MEM mode
 * b) the LCD display HD44780U provides 4 rows with 20 characters each
 * c) the VID:PID is 1A86:5512 - please note that there a more Linux drivers
 *    around working with exactly this 1A86:5512 VID:PID, too; e.g.
 *    https://github.com/gschorcht/spi-ch341-usb
 * d) the driver name is "ch341ahd44780u" meaning USB bridge controller
 *    "ch341a" with LCD display "hd44780u"
 * e) the devices in the /dev tree are named "hd44780u_dpN" where N
 *    starts with 0 and is incremented for each additional display
 *    connected to the host
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/kref.h>
#include <linux/uaccess.h>
//#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/usb.h>

#include <linux/version.h>

#undef pr_fmt

/* #define DEBUG */

#ifdef DEBUG

#define pr_fmt(fmt) KBUILD_MODNAME ": file: " __FILE__ ", function %s, line %d: " fmt, __func__, __LINE__

#else

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#endif

#define CH34x_VENDOR_ID					0x1A86	/* Vendor Id */
#define CH34x_PRODUCT_ID				0x5512	/* Product Id */
#define DRV_NAME                "ch341ahd44780u"

#define CH34x_MINOR_BASE				200
#define WRITES_IN_FLIGHT        8

/* Vendor define */
#define VENDOR_WRITE_TYPE				0x40	/* vendor write command */
#define VENDOR_READ_TYPE				0XC0	/* vendor read command */

#define CH34x_PARA_INIT					0xB1	/* Init Parallel */
#define CH34x_I2C_STATUS				0x52	/* get I2C status */
#define CH34x_I2C_COMMAND				0x53	/* send I2C command */

#define CH34x_BUF_CLEAR					0xB2	/* clear uncompleted data */
#define CH34x_I2C_CMD_X					0x54	/* send I2C command */
#define CH34x_DELAY_MS					0x5E
#define VENDOR_VERSION					0x5F	/* get version of chip */

#define CH34x_PARA_CMD_R0				0xAC	/* read data0 from Para */
#define CH34x_PARA_CMD_R1				0xAD	/* read data1 from Para */
#define CH34x_PARA_CMD_W0				0xA6	/* write data0 to Para */
#define CH34x_PARA_CMD_W1				0xA7	/* write data1 to Para */
#define CH34x_PARA_CMD_STS			0xA0	/* get status of Para */

//CH341 COMMAND
#define CH34x_CMD_SET_OUTPUT			0xA1	/* set Para output */
#define CH34x_CMD_IO_ADDR				  0xA2	/* MEM IO Addr */
#define CH34x_CMD_PRINT_OUT				0xA3	/* print output */
#define CH34X_CMD_SPI_STREAM			0xA8	/* SPI command */
#define CH34x_CMD_SIO_STREAM			0xA9	/* SIO command */
#define CH34x_CMD_I2C_STREAM			0xAA	/* I2C command */
#define CH34x_CMD_UIO_STREAM			0xAB	/* UIO command */

#define	CH341A_CMD_UIO_STM_IN			0x00	/* UIO Interface In ( D0 ~ D7 ) */
#define	CH341A_CMD_UIO_STM_DIR		0x40	/* UIO interface Dir( set dir of D0~D5 ) */
#define	CH341A_CMD_UIO_STM_OUT		0x80	/* UIO Interface Output(D0~D5) */
#define	CH341A_CMD_UIO_STM_US			0xC0	/* UIO Interface Delay Command( us ) */
#define	CH341A_CMD_UIO_STM_END		0x20	/* UIO Interface End Command */

//request
#define CH34x_DEBUG_READ				0x95	/* read two regs */
#define CH34x_DEBUG_WRITE				0x9A	/* write two regs */

#define REQUEST_TYPE_READ				( USB_DIR_IN |USB_TYPE_VENDOR | USB_RECIP_OTHER )
#define REQUEST_TYPE_WRITE				( USB_DIR_OUT | USB_TYPE_VENDOR |USB_RECIP_OTHER)

struct lcddisplay {
	struct usb_device *udev;	/* the usb device for this device */
	struct usb_interface *interface;	/* the interface for this device */
	struct usb_endpoint_descriptor *interrupt_in_endpoint;

	size_t bulk_in_size;	/* the size of rec data (bulk) */
	u8 bulk_in_endpointAddr;	/* bulk input endpoint */
	u8 bulk_out_endpointAddr;	/* bulk output endpoint */

	wait_queue_head_t bulk_event;	/* signalled when bulk out returned from device */
	volatile unsigned int bulk_arrived;

	struct semaphore limit_sem;	/* semaphore */
	struct usb_anchor submitted;	/* usb anchor */

	int errors;
	int open_count;		/* count the number of openers */
	spinlock_t err_lock;
	struct kref kref;
	struct mutex dev_mutex;

	ktime_t last_access;
};

static struct usb_driver lcddisplay_driver;
static void skel_delete(struct kref *kref);

static DEFINE_MUTEX(io_mutex);

/*usb VID/PID Register Into System*/
static struct usb_device_id lcddisplay_usb_ids[] = {
	{USB_DEVICE(CH34x_VENDOR_ID, CH34x_PRODUCT_ID)},
	{}
};

MODULE_DEVICE_TABLE(usb, lcddisplay_usb_ids);

static int lcddisplay_fops_release(struct inode *inode, struct file *file)
{
	struct lcddisplay *dev;

	dev = (struct lcddisplay *)file->private_data;
	if (dev == NULL)
		return -ENODEV;

	mutex_lock(&io_mutex);

	if (!--dev->open_count && dev->interface)
		usb_autopm_put_interface(dev->interface);
	mutex_unlock(&io_mutex);

	kref_put(&dev->kref, skel_delete);
	return 0;
}

//Init Parallel Mode
//iMode-> 00/01 EPP
//iMode-> 02    MEM
static int CH34xInitParallel(unsigned char iMode, struct lcddisplay *dev)
{
	int retval;
	__u8 RequestType = VENDOR_WRITE_TYPE;
	__u8 Request = CH34x_PARA_INIT;
	__u16 Value = (iMode << 8) | (iMode < 0x00000100 ? 0x02 : 0x00);
	__u16 Index = 0;
	__u16 len = 0;
	retval = usb_control_msg(dev->udev,
				 usb_sndctrlpipe(dev->udev, 0), Request,
				 RequestType, Value, Index, NULL, len, 1000);

	return retval;
}

static void skel_delete(struct kref *kref)
{
	struct lcddisplay *dev = container_of(kref, struct lcddisplay, kref);
	usb_put_dev(dev->udev);

	kfree(dev);
}

#define DELAY_MILLISECONDS      2
#define DELAY_MIN_MICROSECONDS  500

/**
 * comment from the author: I do not know why but ktime_get() is NOT
 * reliable as stated e.g. on stackoverflow. We have to wait two (2)
 * milliseconds after each write to the CH341A because we do not read
 * the BF (Busy Flag), which would mean more USB bulk transfers...
 * I started writing code that stores the ktime (in nanoseconds) at
 * the time of the most recent write access.
 * Then, delay_access_conditionally should measure the time spent
 * since the most recent write and conditionally wait for the amount
 * (2 millis - delta time). THIS DOES NOT WORK. Maybe someone can
 * find out what went wrong here... (maybe I am just a 'kernel
 * punk'? :-) ).
 * So currently, it just waits using usleep_range, which might be
 * optimized using the algorithm stated above.
 */
static void delay_access_conditionally(struct lcddisplay *dev)
{
#if 0
	ktime_t this_access = ktime_get();	/* in nanoseconds */
	ktime_t last_access = dev->last_access;
	s64 diff;

	dev->last_access = this_access;

	diff = ktime_to_ns(ktime_sub(this_access, last_access));	// in nanoseconds

	// 2 milliseconds: 2.000.000 nanoseconds

	if (diff < (DELAY_MILLISECONDS * 1000000))	// oops, we have to wait
	{
		diff = (((DELAY_MILLISECONDS * 1000000) - diff) + 500) / 1000;	// diff now in microseconds

		if (diff < DELAY_MIN_MICROSECONDS)
			diff = DELAY_MIN_MICROSECONDS;

		usleep_range(((unsigned long)diff),
			     ((unsigned long)diff) + 100);
	}
#else
	usleep_range(DELAY_MILLISECONDS * 1000,
		     DELAY_MILLISECONDS * 1000 + DELAY_MIN_MICROSECONDS);
#endif
}

//usb_fill_bulk_urb cpmplete callback
static void lcddisplay_write_bulk_callback(struct urb *urb)
{
	struct lcddisplay *dev;
	unsigned long flags;

	dev = urb->context;

	if (unlikely(0 != urb->status)) {
		if (!
		    (urb->status == -ENOENT || urb->status == -ECONNRESET
		     || urb->status == -ESHUTDOWN))
			pr_err("nonzero write bulk status received: %d",
			       urb->status);
		spin_lock_irqsave(&dev->err_lock, flags);
		dev->errors = urb->status;
		spin_unlock_irqrestore(&dev->err_lock, flags);
	}

	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);

	dev->bulk_arrived = 1;
	wake_up_interruptible(&dev->bulk_event);
}

/**
 * @brief This is one of the two main write functions (writing to the CH341A).
 *
 * It takes a 'verb', which is just a 64bit unsigned int. The most significant
 * eight (8) bits contain the number of bytes to be written (obviously from
 * 1 to 7).
 * The remaining seven bytes contain the bytes to be sent to the CH341A and
 * thus to the LCD Display HD44780U in our case.
 *
 * @param[in] dev       the device
 * @param[in] verb      64bit unsigned int containing the number of bytes in
 *                      bits 56..63 and up to seven bytes in the remaining
 *                      bits 0..55 (in Big Endian = Network order), i.e.
 *                      CC B1 B2 B3 B4 B5 B6 B7 (always right shifted).
 *                      Examples:
 *                      02 00 00 00 00 00 A6 30 (write two bytes A6,30)
 *                      03 00 00 00 00 AA 20 30 (write three bytes AA,20,30)
 *
 * @return error code (is a ssize_t but could be an int, too).
 */
ssize_t lcddisplay_write_data(struct lcddisplay *dev, unsigned long verb)
{
	int retval = 0;
	struct urb *urb = NULL;
	unsigned char *buf = NULL;
	size_t l_write = (size_t)(verb >> 56);
	unsigned long flags;

	if (0 == l_write || l_write > 7)	// we can carry up to seven bytes
		return -EFAULT;

	spin_lock_irqsave(&dev->err_lock, flags);
	if ((retval = dev->errors) < 0) {
		dev->errors = 0;
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irqrestore(&dev->err_lock, flags);

	if (retval < 0)
		goto exit;

	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (unlikely(NULL == urb)) {
		retval = -ENOMEM;
		goto error;
	}

	buf =
	    usb_alloc_coherent(dev->udev, l_write, GFP_KERNEL,
			       &urb->transfer_dma);

	if (unlikely(NULL == buf)) {
		retval = -ENOMEM;
		goto error;
	}

	switch (verb >> 56) {
	case 1:
		buf[0] = (unsigned char)verb;
		break;
	case 2:
		buf[0] = (unsigned char)(verb >> 8);
		buf[1] = (unsigned char)verb;
		break;
	case 3:
		buf[0] = (unsigned char)(verb >> 16);
		buf[1] = (unsigned char)(verb >> 8);
		buf[2] = (unsigned char)verb;
		break;
	case 4:
		buf[0] = (unsigned char)(verb >> 24);
		buf[1] = (unsigned char)(verb >> 16);
		buf[2] = (unsigned char)(verb >> 8);
		buf[3] = (unsigned char)verb;
		break;
	case 5:
		buf[0] = (unsigned char)(verb >> 32);
		buf[1] = (unsigned char)(verb >> 24);
		buf[2] = (unsigned char)(verb >> 16);
		buf[3] = (unsigned char)(verb >> 8);
		buf[4] = (unsigned char)verb;
		break;
	case 6:
		buf[0] = (unsigned char)(verb >> 40);
		buf[1] = (unsigned char)(verb >> 32);
		buf[2] = (unsigned char)(verb >> 24);
		buf[3] = (unsigned char)(verb >> 16);
		buf[4] = (unsigned char)(verb >> 8);
		buf[5] = (unsigned char)verb;
		break;
	case 7:
		buf[0] = (unsigned char)(verb >> 48);
		buf[1] = (unsigned char)(verb >> 40);
		buf[2] = (unsigned char)(verb >> 32);
		buf[3] = (unsigned char)(verb >> 24);
		buf[4] = (unsigned char)(verb >> 16);
		buf[5] = (unsigned char)(verb >> 8);
		buf[6] = (unsigned char)verb;
		break;
	default:
		break;
	}

	while (0 != mutex_lock_interruptible(&dev->dev_mutex)) {
	}

	if (!dev->interface) {
		mutex_unlock(&dev->dev_mutex);
		retval = -ENOMEM;
		goto error;
	}

	/*initialize urb */
	usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev,
							  dev->bulk_out_endpointAddr),
			  buf, l_write, lcddisplay_write_bulk_callback, dev);

	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	usb_anchor_urb(urb, &dev->submitted);
	dev->bulk_arrived = 0;
	retval = usb_submit_urb(urb, GFP_KERNEL);

	if (retval) {
		mutex_unlock(&dev->dev_mutex);
		pr_err("failed to write submit urb");
		goto error_unanchor;
	}

	usb_free_urb(urb);

	pr_debug("Waiting for answer to arrive");
	switch (wait_event_interruptible_timeout
		(dev->bulk_event, dev->bulk_arrived != 0,
		 msecs_to_jiffies(2000))) {
	case -ERESTARTSYS:
		pr_debug("answer DID NOT arrive: -ERESTARTSYS");
		break;
	case 0:
		pr_debug("answer DID NOT arrive (timeout)");
		break;
	case 1:
		pr_debug("answer DID arrive but timed out");
		break;
	default:
		pr_debug("answer DID arrive BEFORE timed out (best case)");
		break;
	}

	delay_access_conditionally(dev);

	mutex_unlock(&dev->dev_mutex);

	return 0;

 error_unanchor:
	usb_unanchor_urb(urb);

 error:
	if (urb) {
		usb_free_coherent(dev->udev, l_write, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}

 exit:

	return retval;
}

/**
 * @brief This is the second main write function, which write a full sequence
 *        of commands to the Ch341A (and thus to the HD44780U LCD display).
 *
 * Please also refer to the function 'lcddisplay_write_data' above. The sequence
 * consists of a byte specifying the length of the current unit, followed by the
 * bytes to be sent to the CH341A. Then, the next unit (length byte) follows, etc.
 *
 * @param[in] dev             the device
 * @param[in] p_seq           pointer to buffer containing the sequence
 * @param[in] l_seq           number of bytes in the sequence
 *
 * @return error code (as an ssize_t, could be int, too).
 */
ssize_t lcddisplay_write_sequence(struct lcddisplay *dev, unsigned char *p_seq,
				  unsigned int l_seq)
{
	int retval = 0;
	struct urb *urb = NULL;
	unsigned char *buf = NULL;
	unsigned long flags;
	unsigned int l_cmd;

	/* lock device mutex */

	while (0 != mutex_lock_interruptible(&dev->dev_mutex)) {
	}

	/* work on all commands in the sequence */

	while (0 != l_seq) {
		/* fetch length of command */

		l_cmd = (unsigned int)(*(p_seq++));

		if (unlikely(l_cmd > 32)) {
			mutex_unlock(&dev->dev_mutex);
			return -EINVAL;
		}

		l_seq--;

		if (unlikely(l_cmd > l_seq)) {
			mutex_unlock(&dev->dev_mutex);
			return -EINVAL;
		}

		/* check previous errors - if any */

		spin_lock_irqsave(&dev->err_lock, flags);
		if ((retval = dev->errors) < 0) {
			dev->errors = 0;
			retval = (retval == -EPIPE) ? retval : -EIO;
		}
		spin_unlock_irqrestore(&dev->err_lock, flags);

		if (retval < 0) {
 error:
			mutex_unlock(&dev->dev_mutex);
			return retval;
		}

		/* prepare USB request block (URB) */

		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (unlikely(NULL == urb)) {
			retval = -ENOMEM;
			goto error;
		}

		buf =
		    usb_alloc_coherent(dev->udev, l_cmd, GFP_KERNEL,
				       &urb->transfer_dma);

		if (unlikely(NULL == buf)) {
			retval = -ENOMEM;
			goto error;
		}

		memcpy(buf, p_seq, l_cmd);
		p_seq += l_cmd;
		l_seq -= l_cmd;

		if (!dev->interface) {
			mutex_unlock(&dev->dev_mutex);
			retval = -ENOMEM;
			goto error2;
		}

		/* initialize urb and submit it */

		usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev,
								  dev->bulk_out_endpointAddr),
				  buf, l_cmd, lcddisplay_write_bulk_callback,
				  dev);

		urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
		usb_anchor_urb(urb, &dev->submitted);
		dev->bulk_arrived = 0;
		retval = usb_submit_urb(urb, GFP_KERNEL);

		if (retval) {
			pr_err("failed to write submit urb");
			goto error_unanchor;
		}

		usb_free_urb(urb);

		/* wait for I/O completion, i.e. another BULK OUT answer from the device to the host is expected */

		pr_debug("Waiting for answer to arrive");
		switch (wait_event_interruptible_timeout
			(dev->bulk_event, dev->bulk_arrived != 0,
			 msecs_to_jiffies(2000))) {
		case -ERESTARTSYS:
			pr_debug("answer DID NOT arrive: -ERESTARTSYS");
			break;
		case 0:
			pr_debug("answer DID NOT arrive (timeout)");
			break;
		case 1:
			pr_debug("answer DID arrive but timed out");
			break;
		default:
			pr_debug
			    ("answer DID arrive BEFORE timed out (best case)");
			break;
		}

		delay_access_conditionally(dev);

	}			/* of while command(s) available */

	mutex_unlock(&dev->dev_mutex);

	return 0;

 error_unanchor:
	usb_unanchor_urb(urb);

 error2:
	if (urb) {
		usb_free_coherent(dev->udev, l_cmd, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}

	mutex_unlock(&dev->dev_mutex);

	return retval;
}

int lcddisplay_fops_open(struct inode *inode, struct file *file)
{
	struct lcddisplay *p_display;
	struct usb_interface *interface;
	int retval = 0;
	unsigned int subminor;

	subminor = iminor(file->f_path.dentry->d_inode);

	interface = usb_find_interface(&lcddisplay_driver, subminor);
	if (!interface) {
		retval = -ENODEV;
		goto exit;
	}

	p_display = usb_get_intfdata(interface);
	if (unlikely(NULL == p_display)) {
		retval = -ENODEV;
		goto exit;
	}

	/* add the usage for device */
	kref_get(&p_display->kref);

	mutex_lock(&io_mutex);

	/* perform ops for the very first opening of this device */

	if (!(p_display->open_count++)) {

		retval = usb_autopm_get_interface(interface);

		if (retval) {
 InitError:
			p_display->open_count--;
			mutex_unlock(&io_mutex);
			kref_put(&p_display->kref, skel_delete);
			goto exit;
		}

		/* perform one-time init */

		p_display->last_access = ktime_get();

		pr_debug("Initialize memory mode");
		retval = CH34xInitParallel(0x02, p_display);
		if (0 != retval) {
			pr_err
			    ("unable to switch the USB bridge controller CH341A to MEM mode.");
			goto InitError;
		}

		pr_debug("Send SPI init sequence");
		retval = lcddisplay_write_data(p_display, 0x0300000000AA6100);
		if (0 != retval) {
			pr_err
			    ("unable to send SPI initialization command to USB bridge "
			     "controller CH341A.");
			goto InitError;
		}

		pr_debug("Configure display metrics and font (1/4)");
		/* DL: 8 bits, N: 0 (1 line), Font: 0 (5x8 dots) */
		retval = lcddisplay_write_data(p_display, 0x020000000000A630);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 1/8)");
			goto InitError;
		}

		pr_debug("Configure display metrics and font (2/4)");
		/* DL: 8 bits, N: 0 (1 line), Font: 0 (5x8 dots) */
		retval = lcddisplay_write_data(p_display, 0x020000000000A630);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 2/8)");
			goto InitError;
		}

		pr_debug("Configure display metrics and font (3/4)");
		/* DL: 8 bits, N: 0 (1 line), Font: 0 (5x8 dots) */
		retval = lcddisplay_write_data(p_display, 0x020000000000A630);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 3/8)");
			goto InitError;
		}

		pr_debug("Configure display metrics and font (4/4)");
		/* DL: 8 bits, N: 0 (1 line), Font: 0 (5x8 dots) */
		retval = lcddisplay_write_data(p_display, 0x020000000000A630);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 4/8)");
			goto InitError;
		}

		pr_debug("Configure display metrics and font: 2 lines layout");
		/* DL: 8 bits, N: 1 (2 lines), Font: 0 (5x8 dots) */
		retval = lcddisplay_write_data(p_display, 0x020000000000A638);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 5/8)");
			goto InitError;
		}

		pr_debug
		    ("Configure display off, cursor off, cursor blinking off");
		/* Display off, Cursor off, Cursor blinking off */
		retval = lcddisplay_write_data(p_display, 0x020000000000A608);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 6/8)");
			goto InitError;
		}

		pr_debug("Clear display and set address counter to 0");
		/* Clear entire display, set address counter to 0 */
		retval = lcddisplay_write_data(p_display, 0x020000000000A601);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 7/8)");
			goto InitError;
		}

		pr_debug("Address counter mode: increment, no display shift");
		/* Increment address counter, no display shift */
		retval = lcddisplay_write_data(p_display, 0x020000000000A606);
		if (0 != retval) {
			pr_err
			    ("unable to program the HD44870U display (init 8/8)");
			goto InitError;
		}

		pr_debug("Setup of HD44780U COMPLETE (SUCCESS).");
	}			/* end of one-time initialization */

	file->private_data = p_display;

	mutex_unlock(&io_mutex);

 exit:
	return retval;
}

/**
 * @brief this kernel driver 'understands' two IOCTLs.
 *
 * The first one is 0x80000000, which takes an uint64_t (unsigned long)
 * as the argument interpreting it as a 'verb' to be sent to the CH341A
 * using 'lcddisplay_write_data'.
 * The second one is 0x8xxxxxxx, where xxxxxxx > 0 specifying the
 * number of bytes in a command sequence. The unsigned long (uint64_t)
 * argument is interpreted as a user-space buffer containing the sequence
 * of commands to be written to the CH341A.
 *
 * @param[in] file      the opened LCD display file (/dev/hd44780u_dpN)
 * @param[in] cmd       0x8NNNNNNN with NNNNNNN either 0 (one verb) or
 *                      > 0 (number of bytes in the sequence)
 * @param[in] arg       either the verb (if cmd==0x80000000) or the
 *                      pointer to the user-space buffer containing the
 *                      command sequence)
 *
 * @return error code
 */
long lcddisplay_fops_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	int retval;
	struct lcddisplay *p_display;
	unsigned char *userbuf;

	if (unlikely(NULL == file || 0 == (cmd & 0x80000000 || 0 == arg)))
		return -EFAULT;

	p_display = (struct lcddisplay *)file->private_data;
	if (unlikely(NULL == p_display))
		return -ENODEV;

	if (0x80000000 == cmd) {
		retval = lcddisplay_write_data(p_display, arg);
		if (retval < 0) {
			return -EFAULT;
		}
	} else {
		cmd &= 0x7FFFFFFF;	// size of command queue in bytes

		userbuf = (unsigned char *)kmalloc(cmd, GFP_KERNEL);
		if (unlikely(NULL == userbuf))
			return -ENOMEM;

		if (copy_from_user(userbuf, (const void *)arg, cmd)) {
			kfree(userbuf);
			return -EFAULT;
		}

		retval = lcddisplay_write_sequence(p_display, userbuf, cmd);

		kfree(userbuf);

		if (retval < 0)
			return -EFAULT;
	}

	return 0;
}

static const struct file_operations lcddisplay_fops_driver = {
	.owner = THIS_MODULE,
	.open = lcddisplay_fops_open,
	.release = lcddisplay_fops_release,
	.unlocked_ioctl = lcddisplay_fops_ioctl,
};

/*
 *usb class driver info in order to get a minor number from the usb core
 *and to have the device registered with the driver core
 */
static struct usb_class_driver lcddisplay_class = {
	.name = "hd44780u_dp%d",
	.fops = &lcddisplay_fops_driver,
	.minor_base = CH34x_MINOR_BASE,
};

static int lcddisplay_probe(struct usb_interface *intf, const
			    struct usb_device_id *id)
{
	struct usb_host_interface *hinterface;
	struct usb_endpoint_descriptor *endpoint;
	struct lcddisplay *p_display;

	size_t buffer_size;
	int retval = -ENOMEM;
	int i;

	/* allocate memory for our device state and initialize it */
	p_display = kzalloc(sizeof(*p_display), GFP_KERNEL);
	if (!p_display) {
		pr_err("Out of Memory");
		goto error;
	}

	/* init */
	kref_init(&p_display->kref);
	sema_init(&p_display->limit_sem, WRITES_IN_FLIGHT);
	spin_lock_init(&p_display->err_lock);
	init_usb_anchor(&p_display->submitted);

	p_display->udev = usb_get_dev(interface_to_usbdev(intf));
	p_display->interface = intf;

	hinterface = intf->cur_altsetting;

	if (hinterface->desc.bNumEndpoints < 1)
		return -ENODEV;
	/* Get Endpoint */
	for (i = 0; i < hinterface->desc.bNumEndpoints; ++i) {
		endpoint = &hinterface->endpoint[i].desc;

		if ((endpoint->bEndpointAddress & USB_DIR_IN) &&
		    (endpoint->bmAttributes & 3) == 0x02) {
			pr_debug("Found a bulk in endpoint");
			buffer_size = le16_to_cpu(endpoint->wMaxPacketSize);
			p_display->bulk_in_size = buffer_size;
			p_display->bulk_in_endpointAddr =
			    endpoint->bEndpointAddress;
		}

		if (((endpoint->bEndpointAddress & USB_DIR_IN) == 0x00) &&
		    (endpoint->bmAttributes & 3) == 0x02) {
			pr_debug("Found a bulk out endpoint");
			p_display->bulk_out_endpointAddr =
			    endpoint->bEndpointAddress;
		}

		if ((endpoint->bEndpointAddress & USB_DIR_IN) &&
		    (endpoint->bmAttributes & 3) == 0x03) {
			pr_debug("Found a interrupt in endpoint");
			p_display->interrupt_in_endpoint = endpoint;
		}
	}

	/* save our data point in this interface device */
	usb_set_intfdata(intf, p_display);

	retval = usb_register_dev(intf, &lcddisplay_class);
	if (retval) {
		pr_err("usb_get_dev error, disable to use this device");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	/* perform one-time init */

	init_waitqueue_head(&p_display->bulk_event);
	mutex_init(&p_display->dev_mutex);

	pr_info("HD44780U LCD display now attached to /dev/hd44780u_dp%d",
		intf->minor);

	return 0;

 error:
	if (p_display)
		kref_put(&p_display->kref, skel_delete);

	return retval;
}

static int lcddisplay_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct lcddisplay *dev = usb_get_intfdata(intf);
	int time;

	if (!dev)
		return 0;

	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);

	return 0;
}

static int lcddisplay_resume(struct usb_interface *intf)
{
	return 0;
}

static void lcddisplay_disconnect(struct usb_interface *intf)
{
	struct lcddisplay *dev;
	int minor = intf->minor;

	dev = usb_get_intfdata(intf);
	usb_set_intfdata(intf, NULL);

	/* give back our minor */
	usb_deregister_dev(intf, &lcddisplay_class);

	mutex_lock(&io_mutex);
	dev->interface = NULL;
	mutex_unlock(&io_mutex);

	usb_kill_anchored_urbs(&dev->submitted);
	/*decrement our usage count */
	kref_put(&dev->kref, skel_delete);

	pr_info("HD44780U LCD display now detached from /dev/hd44780u_dp%d",
		minor);
}

static int lcddisplay_pre_reset(struct usb_interface *intf)
{
	struct lcddisplay *dev = usb_get_intfdata(intf);
	int time;

	mutex_lock(&io_mutex);
	time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
	if (!time)
		usb_kill_anchored_urbs(&dev->submitted);

	return 0;
}

static int lcddisplay_post_reset(struct usb_interface *intf)
{
	struct lcddisplay *dev = usb_get_intfdata(intf);

	dev->errors = -EPIPE;
	mutex_unlock(&io_mutex);

	return 0;
}

//usb driver Interface
static struct usb_driver lcddisplay_driver = {
	.name = DRV_NAME,
	.probe = lcddisplay_probe,
	.disconnect = lcddisplay_disconnect,
	.suspend = lcddisplay_suspend,
	.resume = lcddisplay_resume,
	.pre_reset = lcddisplay_pre_reset,
	.post_reset = lcddisplay_post_reset,
	.id_table = lcddisplay_usb_ids,
	.supports_autosuspend = 1,
};

static int __init lcddisplay_init(void)
{
	int retval;
	retval = usb_register(&lcddisplay_driver);
	if (retval)
		pr_err("HD44780U LCD display device registration failed!");
	else
		pr_info("kernel driver for HD44780U LCD display initialized.");
	return retval;
}

static void __exit lcddisplay_exit(void)
{
	usb_deregister(&lcddisplay_driver);
}

module_init(lcddisplay_init);
module_exit(lcddisplay_exit);
MODULE_AUTHOR("Ingo A. Kubbilun (ingo.kubbilun@gmail.com)");
MODULE_DESCRIPTION("CH341A with LCD display HD44780U driver");
MODULE_LICENSE("GPL");
