#include <linux/kernel.h>
#include <linux/module.h>

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>

#include <asm/irq.h>
#include <asm/io.h>
#include <linux/kthread.h>
#include <linux/slab.h>

#include "i2c-inspur-fpga.h"



/* FPGA_i2c_master_complete
 *
 * complete the message and wake up the caller, using the given return code,
 * or zero to mean ok.
 */


static inline void fpga_i2c_master_complete(struct inspur_fpga_i2c *i2c, int ret)
{
	dev_dbg(i2c->dev, "master_complete %d\n", ret);

	i2c->msg_ptr = 0;
	i2c->msg = NULL;
	i2c->msg_idx ++;
	i2c->msg_num = 0;
	if (ret)
		i2c->msg_idx = ret;

	wake_up(&i2c->wait);
}

static inline void fpga_i2c_disable_ack(struct inspur_fpga_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + FPGA_IICCON);
	writel(tmp & ~FPGA_IICCON_ACKEN, i2c->regs + FPGA_IICCON);

}

static inline void fpga_i2c_enable_ack(struct inspur_fpga_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + FPGA_IICCON);
	writel(tmp | FPGA_IICCON_ACKEN, i2c->regs + FPGA_IICCON);

}

/* irq enable/disable functions */

static inline void fpga_i2c_disable_irq(struct inspur_fpga_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + FPGA_IICCON);
	writel(tmp & ~FPGA_IICCON_IRQEN, i2c->regs + FPGA_IICCON);
}

static inline void fpga_i2c_enable_irq(struct inspur_fpga_i2c *i2c)
{
	unsigned long tmp;

	tmp = readl(i2c->regs + FPGA_IICCON);
	writel(tmp | FPGA_IICCON_IRQEN, i2c->regs + FPGA_IICCON);
}


/* fpga_i2c_message_start
 *
 put the start of a message onto the bus 
 */

static int fpga_i2c_message_transfer(struct inspur_fpga_i2c *i2c)
{
	//int i = 0;
	int j = 0;
	int ret = 0;
	unsigned long value = 0;
	unsigned long readdata = 0;
	unsigned long stat;
	unsigned writetemplen = 0;

	struct i2c_msg *msg = i2c->msg;
	unsigned int addr = (msg->addr & 0x7f) << 1;
	char reg_low = msg->buf[0];
	char reg_high = msg->buf[1];
	char wdata = msg->buf[2];
	char wdata1 = 0;
  unsigned int len = 0;


  if (msg->flags & I2C_M_RD) 
    len = msg->len - 2;
  else
    len = 1; //before write, just read one byte for test the slave address
  
  
	//volatile unsigned long *data = (unsigned long *)(i2c->regs + I2C_DATA );
	//volatile unsigned long *status = (unsigned long *)(i2c->regs + I2C_STAT );

	pdebug( "XXX Slave addr:0x%x, offset_low:0x%x, offset_high:0x%x, flags:%s, msg->len:%d, rwlen:%d\n", addr, reg_low, reg_high, (msg->flags&I2C_M_RD) ? "read":"write", msg->len, len);
	pdebug( "XXX Buffer Start:\n");
	for (j = 0; j< msg->len; j++) {
		pdebug( "buf[%d]:0x%x,", j, msg->buf[j]);
		pdebugg( " \n--------------------\n");
	}
	pdebug( "\nXXX Buffer End\n");

	spin_lock_irq(&i2c->lock);
	//i2c->msg_ptr = 2;

    /* first read test for slave response */
		i2c->state = STATE_READ;

    if (0 == reg_high && 0 == reg_low) {
      value = (I2C_START | addr | WRITE) | ((I2C_START | addr | READ) << 16);
      writel(value, i2c->regs + I2C_DATA );
      value = (len | I2C_STOP) | (I2C_INVALID << 16);
      writel(value, i2c->regs + I2C_DATA );
    }else if (0 == reg_high) {
			value = (addr | WRITE | I2C_START) | (reg_low << 16);
			writel(value, i2c->regs + I2C_DATA );
			value = (I2C_START | addr | READ ) | ((I2C_STOP  | len) << 16);
			writel(value, i2c->regs + I2C_DATA );
		} else {
			value = (addr | WRITE | I2C_START) | (reg_high << 16);
			writel(value, i2c->regs + I2C_DATA );
			value = (reg_low) | ((I2C_START | addr | READ) << 16);
			writel(value, i2c->regs + I2C_DATA );
			value = (I2C_STOP | len) | (I2C_INVALID << 16);
      writel(value, i2c->regs + I2C_DATA );
		}

    udelay(270);
		stat = readl(i2c->regs + I2C_STAT);

		/*slave response noack ?*/
		if (stat & FPGA_IICSTAT_NORESP) {
			dev_err(i2c->dev,"Slave is no response!\n");

			ret = -1;
			goto out;
		}


	if (msg->flags & I2C_M_RD) {


		pdebug( "While circle read\n"); 
		while (1) {

			if (stat & FPGA_IICSTAT_HAVEDATA) {
				pdebug( "Can read data, one times tow byte data\n");

				readdata = readl(i2c->regs + I2C_DATA);
				msg->buf[i2c->msg_ptr] = readdata & 0xff;
        pdebug( "circle read %d, content:0x%x\n", i2c->msg_ptr - 2, msg->buf[i2c->msg_ptr]);

        /* judge, avoid buffer is overflow, that is critical */
				if (i2c->msg_ptr < (i2c->msg->len - 1)) {
					i2c->msg_ptr++;
				} else {
					dev_err(i2c->dev,"Slave read byte is odd, out of buffer!\n");
					break;
				}
				
				msg->buf[ i2c->msg_ptr] = readdata >> 16 & 0xff;
        pdebug( "circle read %d, content:0x%x\n", i2c->msg_ptr - 2, msg->buf[i2c->msg_ptr]);

				/* judge again, we can not full trust the hardware,fpga and slave */
				if (i2c->msg_ptr < (i2c->msg->len - 1)) {
					i2c->msg_ptr++;
				} else {
					dev_err(i2c->dev,"Slave read byte is even, out of buffer!\n");
					break;
				}
			} else {
			  udelay(i2c->tx_setup);
        /*slave overtime ?*/
  			stat = readl(i2c->regs + I2C_STAT);
  			if ( stat & FPGA_IICSTAT_OVERTIME) {
  				dev_err(i2c->dev,"Slave read data overtime\n");
  				goto out;
  			}
      }
      
			 stat = readl(i2c->regs + I2C_STAT);

		}


	} else {

    while ( !(stat & FPGA_IICSTAT_HAVEDATA) ) {
      
        udelay(i2c->tx_setup);
        /*slave overtime ?*/
  			stat = readl(i2c->regs + I2C_STAT);
  			if ( stat & FPGA_IICSTAT_OVERTIME) {
  				dev_err(i2c->dev,"Slave read data overtime\n");
  				goto out;
  			}
    }

    pdebug("just read one and will drop it\n");
    readdata = readl(i2c->regs + I2C_DATA);//just read one and drop it
 	
    
		i2c->state = STATE_WRITE;

    if (0 == reg_high && 0 == reg_low) {
      value = (addr | WRITE | I2C_START) | (I2C_INVALID << 16);
      writel(value, i2c->regs + I2C_DATA);
    } else if (0 == reg_high) {
			value = (addr | WRITE | I2C_START) | (reg_low << 16);
			writel(value, i2c->regs + I2C_DATA);
		} else {
			value = (addr | WRITE | I2C_START) | (reg_high << 16);
			writel(value, i2c->regs + I2C_DATA);
			value = (reg_low) | (I2C_INVALID << 16);
      writel(value, i2c->regs + I2C_DATA);

		}

    udelay(270);
		stat = readl(i2c->regs + I2C_STAT);

		/*slave response noack ?*/
		if (stat & FPGA_IICSTAT_NORESP) {
			dev_err(i2c->dev,"Slave is no response!\n");

			ret = -1;
			goto out;
		}

    		/*odd or even*/
		if (msg->len % 2) 
			writetemplen = msg->len - 1;
		else
			writetemplen = msg->len - 2;
    
		/* tow byte tranfer to slave */
		for (; i2c->msg_ptr < writetemplen; i2c->msg_ptr+=2) {
			wdata = msg->buf[i2c->msg_ptr];
			wdata1 = msg->buf[i2c->msg_ptr + 1];
			value = (wdata) | (wdata1 << 16);
			writel(value, i2c->regs + I2C_DATA);
      pdebug( "circle write %d, content1:0x%x, content2:0x%x\n", i2c->msg_ptr - 2, wdata, wdata1);
     
		}


		/*if len is odd, for the last byte*/
		if (msg->len % 2) {
			wdata = msg->buf[i2c->msg_ptr];
			value = (wdata | I2C_STOP) | (I2C_INVALID << 16);
			writel(value, i2c->regs + I2C_DATA);
      pdebug("odd, circle write last byte:0x%x, msg_ptr:%d\n", wdata, i2c->msg_ptr);
		} else {
		  wdata = msg->buf[i2c->msg_ptr];
      wdata1 = msg->buf[i2c->msg_ptr + 1];
		  value = (wdata) | ((wdata1 | I2C_STOP) << 16);
      writel(value, i2c->regs + I2C_DATA);

      i2c->msg_ptr++;
      pdebug("even, circle write last tow byte:0x%x, 0x%x, msg_ptr:%d\n\n", wdata, wdata1, i2c->msg_ptr);
		}

		/*slave overtime ?*/
		stat = readl(i2c->regs + I2C_STAT);
		if ( stat & FPGA_IICSTAT_OVERTIME) {
			dev_err(i2c->dev,"Slave write data overtime\n");
			goto out;
		}


#if 0

		for (; i2c->msg_ptr < msg->len; i2c->msg_ptr++) {
			wdata = msg->buf[i2c->msg_ptr + 2];

			if (0 == reg_high) {
				value = (addr | WRITE | I2C_START) | (reg_low << 16);
				writel(value, i2c->regs + I2C_DATA);
				value = (wdata | I2C_STOP) | (I2C_INVALID << 16);
				writel(value, i2c->regs + I2C_DATA);
			} else {
				value = (addr | WRITE | I2C_START) | (reg_high << 16);
				writel(value, i2c->regs + I2C_DATA);
				value = (reg_low) | ((wdata | I2C_STOP) << 16);
				writel(value, i2c->regs + I2C_DATA);
			}

			/*slave overtime ?*/
			stat = readl(i2c->regs + I2C_STAT);
			if ( stat & FPGA_IICSTAT_OVERTIME) {
				dev_err(i2c->dev,"Slave write data overtime\n");
				goto out;
			}
		} 
#endif
	} 

	pdebug("over read or write, will judeg the bus busy\n");

	/* bus busy ?, if nobusy transfer complete */
	stat = readl(i2c->regs + I2C_STAT);
	if (stat & FPGA_IICSTAT_BUSNOBUSY) {
		i2c->state = STATE_IDLE;
    pdebug("bus no busy\n");
	} else {
		/* delay, waitting for bus no busy, we only can wait for the one times, no more*/
		udelay(i2c->tx_setup);
    stat = readl(i2c->regs + I2C_STAT);
		if (stat & FPGA_IICSTAT_BUSNOBUSY) {
			i2c->state = STATE_IDLE;
      pdebug("after delay bus no busy\n");
    } else 
      dev_err(i2c->dev, "bus alreay busy is status:%d, we can not wait\n", i2c->state);

      pdebug("stat 0x1c value:0x%lx\n", stat);
      stat = readl(i2c->regs + 0x0c);
      pdebug("fifo 0x0c value:0x%lx\n", stat);
      
	}

	/* is last byte */
	if (i2c->msg_ptr == i2c->msg->len - 1 ) {
		/* !last msg, then next msg */
		if ( i2c->msg_idx < i2c->msg_num - 1) {
			i2c->msg_ptr = 2;
	//	i2c->msg_idx++;
			i2c->msg++;
		}
    i2c->msg_idx++;
	}

	goto done;

#if 0
	/* !last byte*/
	if (i2c->msg_ptr < (i2c->msg->len) - 1) {
		i2c->msg_ptr++;
	} else {
		/* !last msg, then next msg */
		if ( i2c->msg_idx < (i2c->msg_num -1)) {
			i2c->msg_ptr = 0;
			i2c->msg_idx++;
			i2c->msg++;
		}
	}

#endif


out:
	i2c->state = STATE_STOP;
	i2c->msg_ptr = 0x0;
	i2c->msg = NULL;
	i2c->msg_idx = 0;
	i2c->msg_num = 0;


done:
	pdebug("complete...\n");
	complete(&i2c->cmd_complete);
	spin_unlock_irq(&i2c->lock);


	//wake_up(&i2c->wait);

	return ret;
}

#if 0
static void fpga_i2c_message_start(struct inspur_fpga_i2c *i2c, 
		struct i2c_msg *msg)
{

	//i2c->state = STATE_START;

	unsigned long stat;
	unsigned long iiccon;
	stat = 0;
	stat |=  FPGA_IICSTAT_TXRXEN;

	if (msg->flags & I2C_M_RD) {
		stat |= FPGA_IICSTAT_MASTER_RX;
		addr |= 1;
	} else
		stat |= FPGA_IICSTAT_MASTER_TX;

	if (msg->flags & I2C_M_REV_DIR_ADDR)
		addr ^= 1;

	// todo - check for wether ack wanted or not
	fpga_i2c_enable_ack(i2c);

	iiccon = readl(i2c->regs + FPGA_IICCON);
	writel(stat, i2c->regs + FPGA_IICSTAT);

	dev_dbg(i2c->dev, "START: %08lx to IICSTAT, %02x to DS\n", stat, addr);
	writeb(addr, i2c->regs + FPGA_IICDS);

	/* delay here to ensure the data byte has gotten onto the bus
	 * before the transaction is started */

	ndelay(i2c->tx_setup);

	dev_dbg(i2c->dev, "iiccon, %08lx\n", iiccon);
	writel(iiccon, i2c->regs + FPGA_IICCON);

	stat |=  FPGA_IICSTAT_START;
	writel(stat, i2c->regs + FPGA_IICSTAT);
}
#endif

static inline void fpga_i2c_stop(struct inspur_fpga_i2c *i2c, int ret)
{
	unsigned long iicstat = readl(i2c->regs + FPGA_IICSTAT);

	dev_dbg(i2c->dev, "STOP\n");

	/* stop the transfer */
	iicstat &= ~ FPGA_IICSTAT_START;
	writel(iicstat, i2c->regs + FPGA_IICSTAT);

	i2c->state = STATE_STOP;

	fpga_i2c_master_complete(i2c, ret);
	fpga_i2c_disable_irq(i2c);
}


#if 0
static int fpga_i2c_set_master(struct inspur_fpga_i2c *i2c)
{
	unsigned long iicstat;
	int timeout = 400;

	while (timeout-- > 0) {
		iicstat = readl(i2c->regs + FPGA_IICSTAT);

		if (!(iicstat & FPGA_IICSTAT_BUSBUSY))
			return 0;

		msleep(1);
	}

	//	dev_dbg(i2c->dev, "timeout: GPEDAT is %08x\n",
	//		__raw_readl(S3C2410_GPEDAT));

	return -ETIMEDOUT;
}
#endif

/* fpga_i2c_set_master
 *
 * get the i2c bus for a master transaction
 */

static int fpga_i2c_set_master(struct inspur_fpga_i2c *i2c)
{
	unsigned long iicstat;
	int timeout = 400;

	while (timeout-- > 0) {
		iicstat = readl(i2c->regs + I2C_STAT);

		if (iicstat & FPGA_IICSTAT_BUSNOBUSY)
			return 0;

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int fpga_i2c_doxfer(struct inspur_fpga_i2c *i2c, struct i2c_msg *msgs, int num)
{
	//unsigned long timeout;
	int ret = 0;
	//int j = 0;
	if (i2c->suspended)
		return -EIO;

#if 0
	pdebug( "msg_num:%d\n", num);
	for (j = 0; j< msgs->len; j++) {
		pdebug( "buf[%d]:0x%x->", j, msgs->buf[j]);
		//pdebug( " \n--------------------\n");
	}
	pdebug( " \n--------------------\n");
#endif

	ret = fpga_i2c_set_master(i2c);
	if (ret != 0) {
		dev_err(i2c->dev, "cannot get bus (error %d)\n", ret);
		ret = -EAGAIN;
		goto out;
	}

	spin_lock_irq(&i2c->lock);
	//mutex_lock(&i2c->lock);
	i2c->msg     = msgs;
	i2c->msg_num = num;
	i2c->msg_ptr = 2;
	i2c->msg_idx = 0;
	i2c->state   = STATE_START;

	//fpga_i2c_enable_irq(i2c);
	//fpga_i2c_message_start(i2c, msgs);


	//mutex_unlock(&i2c->lock);
	wake_up_interruptible(&i2c->wait);
	spin_unlock_irq(&i2c->lock);
	//timeout = wait_event_timeout(i2c->wait, i2c->msg_num == 0, HZ * 5);
	/* wait for tx to complete */
	if (!wait_for_completion_timeout(&i2c->cmd_complete, HZ*5)) {
		dev_err(i2c->dev, "controller timed out\n");
		ret = -ETIMEDOUT;
		goto out;
	}  

	if (i2c->state != STATE_IDLE) {
    pdebug("bus busy, msg_idx:%d, msg_ptr:%d\n", i2c->msg_idx, i2c->msg_ptr);
		ret = i2c->msg_idx;
		if (ret != num) {
			dev_err(i2c->dev, "incomplete msg xfer (%d)\n", ret);
			return -1;
		}

		ret = i2c->msg_ptr + 1;
		if (ret != i2c->msg->len) {
			dev_err(i2c->dev, "incomplete byte xfer (%d)\n", ret);
			return -1;
		}

	}else {
    //ret = (i2c->msg->len - 2) * num;
    ret = i2c->msg_idx;
    pdebug("bus not busy, msg_idx:%d, msg_ptr:%d\n", i2c->msg_idx, i2c->msg_ptr);
  }
  

	/* ensure the stop has been through the bus */
	//msleep(1);

	//s3c24xx_i2c_wait_idle(i2c);

	//s3c24xx_i2c_disable_bus(i2c);

out:
	i2c->state = STATE_IDLE;

	return ret;

#if 0
	int i;
	for (i = 0; i < num; i++) {  
		i2c_adapter_xxx_start();         /*产生起始位*/  
		if (msgs[i]->flags & I2C_M_RD) {    /*读取*/  
			i2c_adapter_xxx_setaddr((msg->addr << 1) | 1);  /*发送从设备地址*/  
			i2c_adapter_xxx_wait_ack();   /*获得从设备的ACK*/  
			i2c_adapter_xxx_readbytes(msgs[i]->buf,msgs[i]->len);  /*读取len长度的数据到buf中*/  
		} else {  
			i2c_adapter_xxx_setaddr(msg->addr << 1);  
			i2c_adapter_xxx_wait_ack();  
			i2c_adapter_xxx_writebytes(msgs[i]->buf, msgs[i]->len);  
		}  
	}  
	i2c_adapter_xxx_stop(); /*产生停止位*/ 
#endif
}

static int i2c_inspur_fpga_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)  
{  
	struct inspur_fpga_i2c *i2c = (struct inspur_fpga_i2c *)adap->algo_data;
	int retry;
	int ret;

	for (retry = 0; retry < adap->retries; retry++) {
		ret = fpga_i2c_doxfer(i2c, msgs, num);
		if (ret != -EAGAIN)
			return ret;

		dev_err(i2c->dev, "Retrying transmission (%d)\n", retry);
		udelay(100);
	}

	return -EREMOTEIO;

}  

static u32 i2c_inspur_fpga_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_PROTOCOL_MANGLING;

}

static struct i2c_algorithm i2c_inspur_fpga_algo = {
	.master_xfer	= i2c_inspur_fpga_xfer,
	.functionality	= i2c_inspur_fpga_func,
};

#if 0
static irqreturn_t fpga_i2c_irq_handler(int irqno, void *dev_id)
{
	struct inspur_fpga_i2c *i2c = (struct inspur_fpga_i2c *)dev_id;

	return IRQ_HANDLED;
}

#endif

static int fpga_i2c_irq_simulate(void *dev_id)
{
	struct inspur_fpga_i2c *i2c = (struct inspur_fpga_i2c *)dev_id;
	int ret = 0;
	while (1) {
		//set_current_state(TASK_INTERRUPTIBLE);
		// io_wait_event(i2c->wait, i2c->state == STATE_START);
		wait_event_interruptible(i2c->wait, i2c->state == STATE_START);
		ret = fpga_i2c_message_transfer(i2c);
		if (!ret) {
			dev_info(i2c->dev, "irq simulate handler transfer ok!\n");
		}

	}
	return ret;
}

#if 0
static int fpga_i2c_init(struct inspur_fpga_i2c *i2c)
{
	unsigned long iicon = FPGA_IICCON_IRQEN | FPGA_IICCON_ACKEN;
	unsigned long iiccon;
	unsigned int freq;

	/* get the plafrom data */
	struct fpga_platform_i2c *pdata = (struct fpga_platform_i2c *)(i2c->adap.dev.parent->platform_data);

	/* write slave address */
	writeb(pdata->slave_addr, i2c->regs + FPGA_IICADD);
	dev_info(i2c->dev, "slave address 0x%02x\n", pdata->slave_addr);

	writel(iicon, i2c->regs + FPGA_IICCON);

	/* we need to work out the divisors for the clock... */
	iiccon = readl(i2c->regs + FPGA_IICCON);
	iiccon &= ~(FPGA_IICCON_SCALEMASK | FPGA_IICCON_TXDIV_512);
	iiccon |= (1-1);

	writel(iiccon, i2c->regs + FPGA_IICCON);
	dev_info(i2c->dev, "bus frequency set to %d KHz\n", freq);
	dev_dbg(i2c->dev, "S3C2410_IICCON=0x%02lx\n", iicon);

	return 0;
}

#endif

#if 0

static struct inspur_fpga_i2c fpga_i2c = {
	.lock		= __SPIN_LOCK_UNLOCKED(fpga_i2c.lock),
	.wait		= __WAIT_QUEUE_HEAD_INITIALIZER(fpga_i2c.wait),
	.tx_setup	= 50,
	.adap		= {
		.name			= "fpga-i2c",
		.owner			= THIS_MODULE,
		.algo			= &i2c_inspur_fpga_algo,
		.retries		= 2,
		.class			= I2C_CLASS_HWMON | I2C_CLASS_SPD,
	},
};
#endif

static int inspur_fpga_i2c_probe(struct platform_device *pdev)
{ 
	int ret = 0;
	struct task_struct *task_thread = NULL;
	struct fpga_platform_i2c *pdata = NULL;
	struct resource *res = NULL;
	/*初始化适配器信息 */ 
	// struct inspur_fpga_i2c *i2c = &fpga_i2c;
	struct inspur_fpga_i2c *i2c;

	i2c = devm_kzalloc(&pdev->dev, sizeof(struct inspur_fpga_i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	strlcpy(i2c->adap.name, "fpga-i2c", sizeof(i2c->adap.name));
	i2c->adap.owner = THIS_MODULE;
	i2c->adap.algo = &i2c_inspur_fpga_algo;
	i2c->adap.retries = 2;
	i2c->adap.class = I2C_CLASS_DEPRECATED;
	i2c->tx_setup = 100; //100ns

	init_waitqueue_head(&i2c->wait);
	init_completion(&i2c->cmd_complete);
	spin_lock_init(&i2c->lock);

	i2c->dev = &pdev->dev;
	/*映射寄存器 */ 

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot find IO resource\n");
		ret = -ENOENT;
		goto err_no_ioresource;
	}

	i2c->ioarea = request_mem_region(res->start, (res->end-res->start)+1,
			pdev->name);

	if (i2c->ioarea == NULL) {
		dev_err(&pdev->dev, "cannot request IO\n");
		ret = -ENXIO;
		goto err_no_mem_region;
	}

	i2c->regs = ioremap(res->start, (res->end-res->start)+1);

	if (i2c->regs == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		ret = -ENXIO;
		goto err_ioarea;
	}

	/*设置I2C核心需要的信息 */  
	i2c->adap.algo_data= i2c;  
	i2c->adap.dev.parent= &pdev->dev;  

#if 0
	/*初始化I2C控制器 */  
	ret= fpga_i2c_init(i2c);  
	if (ret != 0)
		goto err_iomap;

	/*申请中断 */  
	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot find IRQ\n");
		goto err_iomap;
	}

	i2c->irq = ret;

	ret= request_irq(i2c->irq->start, fpga_i2c_irq_handler, 0, dev_name(&pdev->dev), i2c);  
	if (ret != 0) {
		dev_err(&pdev->dev, "cannot claim IRQ\n");
		goto err_iomap;
	}
#endif

	task_thread = kthread_run(fpga_i2c_irq_simulate, i2c, "simu_i2c_irq-%d", pdev->id);
	if(IS_ERR(task_thread)){
		dev_err(&pdev->dev, "Unable to start kernel thread.\n");
		ret = PTR_ERR(task_thread);
		task_thread = NULL;
		goto err_iomap;;
	}

	/* 注册I2C适配器 */
	if (pdev->dev.platform_data != NULL) {
		//pdata = (struct fpga_platform_i2c *)pdev->dev.platform_data;
		pdata =dev_get_platdata(&pdev->dev);
		i2c->adap.nr = pdata->bus_num; 
	} else {
		i2c->adap.nr = -2;
		dev_err(&pdev->dev, "no platform data\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, i2c);

	ret= i2c_add_numbered_adapter(&i2c->adap);  
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add bus to i2c core\n");
		goto err_irq;
	}

	//dev_info(&pdev->dev, "%s: inspur fpga I2C adapte\n", dev_name(&i2c->adap.dev));
	dev_info(&pdev->dev,  "%s: inspur fpga I2C adapter.\n", dev_name(&i2c->adap.dev));

	//	dev_info(&pdev->dev, "%s: inspur fpga I2C adapter\n", i2c->adap.dev.bus_id);

	return 0;



err_irq:
	//free_irq(i2c->irq->start, i2c);

err_iomap:
	iounmap(i2c->regs);

err_ioarea:
	release_resource(i2c->ioarea);
	kfree(i2c->ioarea);

err_no_mem_region:
err_no_ioresource:
	devm_kfree(&pdev->dev, i2c);

	return ret;
}

static int inspur_fpga_i2c_remove(struct platform_device *pdev)
{
	struct inspur_fpga_i2c *i2c = platform_get_drvdata(pdev);
	i2c_del_adapter(&i2c->adap);

	return 0;
}

static const struct platform_device_id inspur_i2c_ids[]= {
	[0] = {
		.name = "CPU-FPGA",
		.driver_data = 0,
	},
	[1] = {
		.name = "FAN-PSOC",
		.driver_data = 0,
	},
	[2] = {
		.name = "FRU",
		.driver_data = 0,
	},
	[3] = {
		.name = "PSU",
		.driver_data = 0,
	},
	[4] = {
		.name = "PCH",
		.driver_data = 0,
	},
	[5] = {
		.name = "TEMPERATURE",
		.driver_data = 0,
	},
	[6] = {
		.name = "SAS-HD",
		.driver_data = 0,
	},
};

static struct platform_driver inspur_fpga_i2c_driver = {
	.probe		= inspur_fpga_i2c_probe,
	.remove		= inspur_fpga_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "inspur-fpga-i2c",
	},
	.id_table	= inspur_i2c_ids,
};

/*below is fpga i2c device attribute */
static struct fpga_platform_i2c fpga_i2c_default_platform = {
	.bus_num  = -1,
	.flags		= 0,
	//.slave_addr	= 0x10,
	//.bus_freq	= 100*1000,
	//.max_freq	= 400*1000,
};

#define RESOURCE_MEM(mem_base, mem_size, mem_name)	\
{								\
	.start = mem_base, \
	.end = mem_base + mem_size,\
	.name = mem_name,\
	.flags	= IORESOURCE_MEM|IORESOURCE_EXCLUSIVE,		\
}

#define RESOURCE_IRQ(irq_base, irq_size)	\
{								\
	.start = irq_base, \
	.end = irq_base + irq_size,\
	.name = "inspur-fpga-i2c-irq-0",\
	.flags	= IORESOURCE_IRQ,		\
}



static struct resource fpga_i2c_resource[][2] = {
	[0] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_3, FPGA_SZ_IIC, MEM_NAME0),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	}, 
	[1] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_4, FPGA_SZ_IIC, MEM_NAME1),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	},   
	[2] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_5, FPGA_SZ_IIC, MEM_NAME2),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	},
	[3] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_6, FPGA_SZ_IIC, MEM_NAME3),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	},
	[4] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_7, FPGA_SZ_IIC, MEM_NAME4),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	},
	[5] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_8, FPGA_SZ_IIC, MEM_NAME5),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	},
	[6] = {
		RESOURCE_MEM(FPGA_PA_IIC + SMBUS_9, FPGA_SZ_IIC, MEM_NAME6),
		RESOURCE_IRQ(FPGA_IRQ_IIC, FPGA_IRQ_IIC)
	}
};


static struct platform_device inspur_fpga_i2c_device[] = {
	[0] = {
		.name = "CPU-FPGA",
		.id = 0,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[0]),
		.resource = fpga_i2c_resource[0],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[1] = {
		.name = "FAN-PSOC",
		.id = 1,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[1]),
		.resource = fpga_i2c_resource[1],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[2] = {
		.name = "FRU",
		.id = 2,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[2]),
		.resource = fpga_i2c_resource[2],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[3] = {
		.name = "PSU",
		.id = 3,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[3]),
		.resource = fpga_i2c_resource[3],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[4] = {
		.name = "PCH",
		.id = 4,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[4]),
		.resource = fpga_i2c_resource[4],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[5] = {
		.name = "TEMPERATURE",
		.id = 5,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[5]),
		.resource = fpga_i2c_resource[5],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	},
	[6] = {
		.name = "SAS-HD",
		.id = 6,
		.num_resources = ARRAY_SIZE(fpga_i2c_resource[6]),
		.resource = fpga_i2c_resource[6],
		.dev		= {
			.platform_data = &fpga_i2c_default_platform,
		},
	}
};

#if 0
static struct resource fpga_i2c_resource1[] = {
	[0] = {
		.start = FPGA_PA_IIC,
		.end = FPGA_PA_IIC + FPGA_SZ_IIC,
		.name	= "inspur-fpga-i2c-1-0",
		.flags	= IORESOURCE_MEM,
	},

	[1] = {
		.start = FPGA_IRQ_IIC,
		.end   = FPGA_IRQ_IIC,
		.flags = IORESOURCE_IRQ,
	}
};



static struct platform_device inspur_fpga_i2c_device1 = {
	.name = "inspur-fpga-i2c",
	.id = -1;
	.num_resources = ARRAY_SIZE(fpga_i2c_resource1),
		.resource = fpga_i2c_resource1,

};

#endif

static struct platform_device *inspur_fpga_i2c_devices[] __initdata = {
	&inspur_fpga_i2c_device[0],
	&inspur_fpga_i2c_device[1],
	&inspur_fpga_i2c_device[2],
	&inspur_fpga_i2c_device[3],
	&inspur_fpga_i2c_device[4],
	&inspur_fpga_i2c_device[5],
	&inspur_fpga_i2c_device[6]
};

static int __init i2c_inspur_fpga_init(void)
{
	int ret;
	int num = ARRAY_SIZE(inspur_fpga_i2c_devices);
	ret = platform_add_devices(inspur_fpga_i2c_devices, num);
	if (0 == ret) {
		ret = platform_driver_register(&inspur_fpga_i2c_driver);
		if (ret) {
			while (--num >= 0)
				platform_device_unregister(inspur_fpga_i2c_devices[num]);
		}
	}


	return ret;
}

subsys_initcall(i2c_inspur_fpga_init);

static void __exit i2c_inspur_fpga_exit(void)
{
	int num = ARRAY_SIZE(inspur_fpga_i2c_devices);
	platform_driver_unregister(&inspur_fpga_i2c_driver);
	while (--num >= 0)
		platform_device_unregister(inspur_fpga_i2c_devices[num]);
}

module_exit(i2c_inspur_fpga_exit);

MODULE_DESCRIPTION("FPGA I2C Bus driver");
MODULE_AUTHOR("Xiaochun Lee, <lixcbj@inspur.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:inspur-fpga-i2c");


