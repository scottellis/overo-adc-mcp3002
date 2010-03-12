/*
  adc.c
 
  Copyright Scott Ellis, 2010
 
  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
 
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 
  Just fooling around exploring features of the Linux OMAP3 SPI driver
  framework. The test devices are Micrologic MCP3002 10-bit ADCs.
*/
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/cdev.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/list.h>
#include <linux/workqueue.h>

/* averaging four reads, no particular reason */
#define NUM_TRANSFERS	4
#define NUM_DEVICES	2

#define SPI_BUFF_SIZE	(NUM_TRANSFERS * 2)
#define USER_BUFF_SIZE	128

/* 
At 5v the max speed the MCP3002's can handle is 200k samples/sec.
At 16 bits per sample this translates to 3.2M as the max clock speed.
The McSPI controller available speeds are
48M / (1 << 1) -> 48M
48M / (1 << 2) -> 24M
48M / (1 << 3) -> 12M
48M / (1 << 4) -> 6M
48M / (1 << 5) -> 3M
...
48M / (1 << 12) -> 11.7K

So 3M is the best we can do.
*/
#define BASE_BUS_SPEED 3000000

static int bus_speed = BASE_BUS_SPEED;
module_param(bus_speed, int, S_IRUGO);
MODULE_PARM_DESC(bus_speed, "SPI bus speed in Hz");

static int running = 0;

struct adc_message {
	u32 adc_id;
	u32 avg;
	struct list_head list;
	struct completion completion;
	struct spi_message msg;
	struct spi_transfer *transfer;
	u8 *rx_buff;
	u8 *tx_buff; 
};

struct adc_dev {
	struct semaphore spi_sem;
	struct semaphore fop_sem;
	dev_t devt;
	struct cdev cdev;
	struct spi_device *spi_device[NUM_DEVICES];	
	struct adc_message adc_msg[NUM_DEVICES];
	char *user_buff;
};

static struct adc_dev adc_dev;

static LIST_HEAD(done_list);
static LIST_HEAD(work_list);
static DEFINE_MUTEX(list_lock);

static void adc_async_complete(void *arg);
static int adc_async(struct adc_message *adc_msg);

static void adc_workq_handler(struct work_struct *work)
{
	int i;
	u32 avg;
	struct adc_message *adc_msg;
	struct adc_message *next;

	/* 
	  get everything out of the done_list and into the work_list
	  so we don't hold up adc_async_complete() with the list_lock
	*/
	mutex_lock(&list_lock);
	list_for_each_entry_safe(adc_msg, next, &done_list, list) {
		list_del_init(&adc_msg->list);
		list_add_tail(&adc_msg->list, &work_list);	
	}
	mutex_unlock(&list_lock);

	/* now we can process the work_list at our leisure */
	list_for_each_entry_safe(adc_msg, next, &work_list, list) {
		list_del_init(&adc_msg->list);

		avg = 0;
		for (i = 0; i < NUM_TRANSFERS * 2; i += 2)
			avg += 0x3ff & ((adc_msg->rx_buff[i] << 8) 
						+ adc_msg->rx_buff[i+1]);	
			
		if (down_interruptible(&adc_dev.spi_sem))
			return;

		adc_msg->avg  = avg / NUM_TRANSFERS;
		up(&adc_dev.spi_sem);

		/* resubmit the message */		
		if (running)
			if (adc_async(adc_msg))
				running = 0;
	}
}

DECLARE_WORK(spi_work, adc_workq_handler);

static void adc_async_complete(void *arg)
{
	struct adc_message *adc_msg = (struct adc_message *) arg;
	
	mutex_lock(&list_lock);
	list_add_tail(&adc_msg->list, &done_list);
	mutex_unlock(&list_lock);

	schedule_work(&spi_work);

	complete(&adc_msg->completion);
}

static int adc_async(struct adc_message *adc_msg)
{
	int i, status;
	struct spi_message *message;
	struct spi_device *spi_device;

	if (down_interruptible(&adc_dev.spi_sem))
		return -EFAULT;

	spi_device = adc_dev.spi_device[adc_msg->adc_id];

	if (spi_device == NULL) {
		printk(KERN_ALERT "adc_async(): spi_device is NULL\n");
		status = -ESHUTDOWN;
		goto adc_async_done;
	}

	INIT_COMPLETION(adc_msg->completion);

	message = &adc_msg->msg;
	spi_message_init(message);
	message->complete = adc_async_complete;
	message->context = adc_msg;
	
	memset(adc_msg->tx_buff, 0, NUM_TRANSFERS * 2);		
	memset(adc_msg->rx_buff, 0, NUM_TRANSFERS * 2);		
	
	adc_msg->tx_buff[0] = 0x40;
	
	memset(adc_msg->transfer, 0, NUM_TRANSFERS * sizeof(struct spi_transfer));

	for (i = 0; i < NUM_TRANSFERS; i++) {
		adc_msg->transfer[i].tx_buf = adc_msg->tx_buff;
		adc_msg->transfer[i].rx_buf = &adc_msg->rx_buff[i*2];
		adc_msg->transfer[i].len = 2;

		/* we need the CS raised between each transfer (measurement) */
		adc_msg->transfer[i].cs_change = 1;
		
		/* override the bus speed if needed */
		if (spi_device->max_speed_hz != bus_speed)
			adc_msg->transfer[i].speed_hz = bus_speed;
		
		/* put in a test delay between messages, signal analyzer debug stuff */
		/*
		if (i == NUM_TRANSFERS - 1)
			adc_msg->transfer[i].delay_usecs = 50;
		*/

		spi_message_add_tail(&adc_msg->transfer[i], message);
	}
			
	status = spi_async(spi_device, message);

adc_async_done:

	up(&adc_dev.spi_sem);

	return status;
}

static ssize_t adc_write(struct file *filp, const char __user *buff,
		size_t count, loff_t *f_pos)
{
	ssize_t	status;
	size_t len;
	int i;

	if(down_interruptible(&adc_dev.fop_sem))
		return -ERESTARTSYS;

	memset(adc_dev.user_buff, 0, 32);
	len = count > 8 ? 8 : count;

	if (copy_from_user(adc_dev.user_buff, buff, len)) {
		status = -EFAULT;
		goto adc_write_done;
	}

	status = count;

	/* we accept two commands, "on" or "off" and ignore anything else*/
	if (!running && !strnicmp(adc_dev.user_buff, "on", 2)) {
		for (i = 0; i < NUM_DEVICES; i++) {
			status = adc_async(&adc_dev.adc_msg[i]);
			if (status) {
				printk(KERN_ALERT 
					"adc_write(): adc_async() returned %d\n",
					status);
				break;
			} else {
				running = 1; 
				status = count;
			}
		}
	} else if (!strnicmp(adc_dev.user_buff, "off", 3)) {
		running = 0;
	}

adc_write_done:
	up(&adc_dev.fop_sem);

	return status;
}

static ssize_t adc_read(struct file *filp, char __user *buff, size_t count,
			loff_t *offp)
{
	size_t len;
	char temp[8];
	int i;
	ssize_t status = 0;

	if (!buff) 
		return -EFAULT;

	/* tell the user there is no more */
	if (*offp > 0) 
		return 0;

	if (down_interruptible(&adc_dev.fop_sem)) 
		return -ERESTARTSYS;

	if (running) {
		strcpy(adc_dev.user_buff, "ADC:");

		for (i = 0; i < NUM_DEVICES; i++) {
			sprintf(temp, " %u", adc_dev.adc_msg[i].avg);
			strcat(adc_dev.user_buff, temp);
		}

		strcat(adc_dev.user_buff, "\n");
	} else {
		strcpy(adc_dev.user_buff, "ADC: off\n");
	}

	len = strlen(adc_dev.user_buff);
 
	if (len < count) 
		count = len;

	if (copy_to_user(buff, adc_dev.user_buff, count))  {
		printk(KERN_ALERT "adc_read(): copy_to_user() failed\n");
		status = -EFAULT;
	} else {
		*offp += count;
		status = count;
	}

	up(&adc_dev.fop_sem);

	return status;	
}

static int adc_open(struct inode *inode, struct file *filp)
{	
	int status = 0;

	if (down_interruptible(&adc_dev.fop_sem)) 
		return -ERESTARTSYS;

	if (!adc_dev.user_buff) {
		adc_dev.user_buff = kmalloc(USER_BUFF_SIZE, GFP_KERNEL);
		if (!adc_dev.user_buff) 
			status = -ENOMEM;
	}	

	up(&adc_dev.fop_sem);

	return status;
}

static int adc_probe(struct spi_device *spi_device)
{
	struct adc_message *adc_msg;
	int status = 0;

	if (down_interruptible(&adc_dev.spi_sem))
		return -EBUSY;

	if (spi_device->chip_select >= 0 && 
		spi_device->chip_select < NUM_DEVICES) {
		
		adc_dev.spi_device[spi_device->chip_select] = spi_device;

		adc_msg = &adc_dev.adc_msg[spi_device->chip_select];
		adc_msg->adc_id = spi_device->chip_select;
		
		init_completion(&adc_msg->completion);

		if (!adc_msg->transfer) {
			adc_msg->transfer = 
				kmalloc(NUM_TRANSFERS * sizeof(struct spi_transfer), 
					GFP_KERNEL);
			if (!adc_msg->transfer) 
				status = -ENOMEM;				
		}
	
		if (!adc_msg->tx_buff) {
			adc_msg->tx_buff = 
				kmalloc(SPI_BUFF_SIZE * sizeof(u8), GFP_KERNEL);
			if (!adc_msg->tx_buff) 
				status = -ENOMEM;
		}

		if (!adc_msg->rx_buff) {
			adc_msg->rx_buff = 
				kmalloc(SPI_BUFF_SIZE * sizeof(u8), GFP_KERNEL);
			if (!adc_msg->rx_buff) 
				status = -ENOMEM;
		}
	} else {
		status = -ENODEV;
	}

	if (!status) 
		printk(KERN_ALERT "SPI[%d] max_speed_hz %d Hz  bus_speed %d Hz\n", 
			spi_device->chip_select, 
			spi_device->max_speed_hz, 
			bus_speed);
	
	up(&adc_dev.spi_sem);

	return status;
}

static int adc_remove(struct spi_device *spi_device)
{
	struct adc_message *adc_msg;

	if (down_interruptible(&adc_dev.spi_sem))
		return -EBUSY;
	
	if (spi_device->chip_select >= 0 && 
		spi_device->chip_select < NUM_DEVICES) {

		adc_dev.spi_device[spi_device->chip_select] = NULL;
		adc_msg = &adc_dev.adc_msg[spi_device->chip_select];

		if (adc_msg->transfer) 
			kfree(adc_msg->transfer);

		if (adc_msg->rx_buff)
			kfree(adc_msg->rx_buff);

		if (adc_msg->tx_buff)
			kfree(adc_msg->tx_buff);
	}

	up(&adc_dev.spi_sem);

	return 0;
}

static int __init add_adc_device_to_bus(void)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	int status;
	int i;
	char buff[64];

	spi_master = spi_busnum_to_master(1);

	if (!spi_master) {
		printk(KERN_ALERT "spi_busnum_to_master(1) returned NULL\n");
		printk(KERN_ALERT "Missing modprobe omap2_mcspi?\n");
		return -1;
	}

	for (i = 0; i < NUM_DEVICES; i++) {
		spi_device = spi_alloc_device(spi_master);

		if (!spi_device) {
			status = -1;
			printk(KERN_ALERT "spi_alloc_device() failed\n");
			break;
		}

		spi_device->chip_select = i;

		/* first check if the bus already knows about us */
		snprintf(buff, sizeof(buff), "%s.%u", dev_name(&spi_device->master->dev),
			spi_device->chip_select);

		if (bus_find_device_by_name(spi_device->dev.bus, NULL, buff)) {
			/* we are already registered, nothing to do, just free the spi_device 
			   this crashes unless you have a patched omap2_mcspi_cleanup() */
			spi_dev_put(spi_device);
			status = 0;
		} else {
			spi_device->max_speed_hz = bus_speed;
			spi_device->mode = SPI_MODE_0;
			spi_device->bits_per_word = 8;
			spi_device->irq = -1;
			spi_device->controller_state = NULL;
			spi_device->controller_data = NULL;
			strlcpy(spi_device->modalias, "adc", SPI_NAME_SIZE);
			status = spi_add_device(spi_device);
		
			if (status < 0) {	
				/* this will crash you unless you have patched omap2_mcspi_cleanup() */	
				spi_dev_put(spi_device);
				printk(KERN_ALERT "spi_add_device() failed: %d\n", status);		
			}				
		}
	}

	put_device(&spi_master->dev);

	return status;
}

static struct spi_driver adc_spi = {
	.driver = {
		.name =	"adc",
		.owner = THIS_MODULE,
	},
	.probe = adc_probe,
	.remove = __devexit_p(adc_remove),	
};

static int __init adc_spi_setup(void)
{
	int error;

	error = spi_register_driver(&adc_spi);
	if (error < 0) {
		printk(KERN_ALERT "spi_register_driver() failed %d\n", error);
		return -1;
	}

	error = add_adc_device_to_bus();
	if (error < 0) {
		printk(KERN_ALERT "add_adc_to_bus() failed\n");
		spi_unregister_driver(&adc_spi);		
	}

	return error;
}

static const struct file_operations adc_fops = {
	.owner =	THIS_MODULE,
	.read = 	adc_read,
	.write =	adc_write,
	.open =		adc_open,	
};

static int __init adc_cdev_setup(void)
{
	int error;

	adc_dev.devt = MKDEV(0, 0);

	if ((error = alloc_chrdev_region(&adc_dev.devt, 0, 1, "adc")) < 0) {
		printk(KERN_ALERT "alloc_chrdev_region() failed: error = %d \n", 
			error);
		return -1;
	}

	cdev_init(&adc_dev.cdev, &adc_fops);
	adc_dev.cdev.owner = THIS_MODULE;
	adc_dev.cdev.ops = &adc_fops;

	error = cdev_add(&adc_dev.cdev, adc_dev.devt, 1);
	if (error) {
		printk(KERN_ALERT "cdev_add() failed: error = %d\n", error);
		unregister_chrdev_region(adc_dev.devt, 1);
		return -1;
	}	

	return 0;
}

static int __init adc_init(void)
{
	memset(&adc_dev, 0, sizeof(struct adc_dev));

	sema_init(&adc_dev.spi_sem, 1);
	sema_init(&adc_dev.fop_sem, 1);
	
	if (adc_cdev_setup() < 0) {
		printk(KERN_ALERT "adc_cdev_setup() failed\n");
		goto fail_1;
	}
	
	if (adc_spi_setup() < 0) {
		printk(KERN_ALERT "adc_spi_setup() failed\n");
		goto fail_2;
	}

	printk(KERN_ALERT "Run : mknod /dev/adc c %d %d\n", 
			MAJOR(adc_dev.devt), MINOR(adc_dev.devt));

	return 0;

fail_2:
	cdev_del(&adc_dev.cdev);
	unregister_chrdev_region(adc_dev.devt, 1);

fail_1:
	return -1;
}

static void __exit adc_exit(void)
{
	spi_unregister_driver(&adc_spi);
	cdev_del(&adc_dev.cdev);
	unregister_chrdev_region(adc_dev.devt, 1);

	if (adc_dev.user_buff)
		kfree(adc_dev.user_buff);
}

module_init(adc_init);
module_exit(adc_exit);

MODULE_AUTHOR("Scott Ellis");
MODULE_DESCRIPTION("SPI OMAP3 experimental ADC driver");
MODULE_LICENSE("GPL");

