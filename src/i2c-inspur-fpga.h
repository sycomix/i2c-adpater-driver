
#ifndef __ASM_FPGA_REGS_IIC_H
#define __ASM_FPGA_REGS_IIC_H

/* handy sizes */
#define SZ_16				0x00000010
#define SZ_256				0x00000100
#define SZ_512				0x00000200

#define SZ_1K                           0x00000400
#define SZ_4K                           0x00001000
#define SZ_8K                           0x00002000
#define SZ_16K                          0x00004000
#define SZ_64K                          0x00010000
#define SZ_128K                         0x00020000
#define SZ_256K                         0x00040000
#define SZ_512K                         0x00080000

#define SZ_1M                           0x00100000
#define SZ_2M                           0x00200000
#define SZ_4M                           0x00400000
#define SZ_8M                           0x00800000
#define SZ_16M                          0x01000000
#define SZ_32M                          0x02000000
#define SZ_64M                          0x04000000
#define SZ_128M                         0x08000000
#define SZ_256M                         0x10000000
#define SZ_512M                         0x20000000

#define SZ_1G                           0x40000000
#define SZ_2G                           0x80000000

#define MEM_NAME0 "inspur-fpga-i2c-0"
#define MEM_NAME1 "inspur-fpga-i2c-1"
#define MEM_NAME2 "inspur-fpga-i2c-2"
#define MEM_NAME3 "inspur-fpga-i2c-3"
#define MEM_NAME4 "inspur-fpga-i2c-4"
#define MEM_NAME5 "inspur-fpga-i2c-5"
#define MEM_NAME6 "inspur-fpga-i2c-6"

/* IIC hardware controller */
#define FPGA_PA_IIC	   (0xff240000)
#define FPGA_SZ_IIC	  (0xff)

#define I2C_STAT        (0x1c)
#define I2C_DATA    (0x08)

#define READ    1
#define WRITE   0
#define I2C_START       2<<8
#define I2C_STOP        1<<8
#define I2C_INVALID     4<<8

#define SMBUS_3 0x0
#define SMBUS_4 0x100
#define SMBUS_5 0x200
#define SMBUS_6 0x300
#define SMBUS_7 0x400
#define SMBUS_8 0x500
#define SMBUS_9 0x600

#define FPGA_IICREG(x) (x)

#define FPGA_IRQ_IIC   FPGA_IICREG(27)
#define FPGA_IICCON    FPGA_IICREG(0x00)
#define FPGA_IICSTAT   FPGA_IICREG(0x04)
#define FPGA_IICADD    FPGA_IICREG(0x08)
#define FPGA_IICDS     FPGA_IICREG(0x0C)
#define FPGA_IICLC	  FPGA_IICREG(0x10)

#define FPGA_IICCON_ACKEN		(1<<7)
#define FPGA_IICCON_TXDIV_16		(0<<6)
#define FPGA_IICCON_TXDIV_512	(1<<6)
#define FPGA_IICCON_IRQEN		(1<<5)
#define FPGA_IICCON_IRQPEND		(1<<4)
#define FPGA_IICCON_SCALE(x)		((x)&15)
#define FPGA_IICCON_SCALEMASK	(0xf)

#define FPGA_IICSTAT_MASTER_RX	(2<<6)
#define FPGA_IICSTAT_MASTER_TX	(3<<6)
#define FPGA_IICSTAT_SLAVE_RX	(0<<6)
#define FPGA_IICSTAT_SLAVE_TX	(1<<6)
#define FPGA_IICSTAT_MODEMASK	(3<<6)

#define FPGA_IICSTAT_START		(1<<5)
  #define FPGA_IICSTAT_BUSNOBUSY	(1<<0)
  #define FPGA_IICSTAT_OVERTIME		(1<<6)
  #define FPGA_IICSTAT_NORESP		  (1<<5)
  #define FPGA_IICSTAT_HAVEDATA		(1<<1)
#define FPGA_IICSTAT_ADDR0		(1<<1)
#define FPGA_IICSTAT_LASTBIT		(1<<0)

#define FPGA_IICLC_SDA_DELAY0	(0 << 0)
#define FPGA_IICLC_SDA_DELAY5	(1 << 0)
#define FPGA_IICLC_SDA_DELAY10	(2 << 0)
#define FPGA_IICLC_SDA_DELAY15	(3 << 0)
#define FPGA_IICLC_SDA_DELAY_MASK	(3 << 0)

#define FPGA_IICLC_FILTER_ON		(1<<2)



struct fpga_platform_i2c {
	int		bus_num;	/* bus number to use */
	unsigned int	flags;
	unsigned int	slave_addr;	/* slave address for controller */
	unsigned long	bus_freq;	/* standard bus frequency */
	unsigned long	max_freq;	/* max frequency for the bus */
	unsigned long	min_freq;	/* min frequency for the bus */
};

enum inspur_fpga_i2c_state {
	STATE_IDLE,
	STATE_START,
	STATE_READ,
	STATE_WRITE,
	STATE_STOP
};

struct inspur_fpga_i2c {
	spinlock_t		lock;
	wait_queue_head_t	wait;
    struct completion	cmd_complete;
	unsigned int		suspended:1;

	struct i2c_msg		*msg;
	unsigned int		msg_num;
	unsigned int		msg_idx;
	unsigned int		msg_ptr;

	unsigned int		tx_setup;

	enum inspur_fpga_i2c_state	state;
	unsigned long		clkrate;

	void __iomem		*regs;
	struct clk		*clk;
	struct device		*dev;
	struct resource		*irq;
	struct resource		*ioarea;
	struct i2c_adapter	adap;

};

#define INSPUR_I2C_DEBUG  1
#undef pdebug             /* undef it, just in case */
#ifdef INSPUR_I2C_DEBUG
  #ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
    #define pdebug(fmt, args...) printk( KERN_ERR "INSPUR-I2C: " fmt, ## args)
    #define pdebugg(fmt, args...) printk( KERN_ERR  fmt, ## args)
  #else
     /* This one for user space */
    #define pdebug(fmt, args...) fprintf(stderr, fmt, ## args)
  #endif
#else
#  define pdebug(fmt, args...) /* not debugging: nothing */
#endif

//#undef PDEBUGG
//#define pdebugg(fmt, args...) /* nothing: it's a placeholder */



#endif /* __ASM_FPGA_REGS_IIC_H */

