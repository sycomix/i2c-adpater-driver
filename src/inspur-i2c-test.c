#include <sys/types.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>



/* MACRO definitions */
#define ABS(X) ((X) < 0 ? (-1 * (X)) : (X))
#define MAX(X, Y) ((X) >= (Y) ? (X) : (Y))
#define MIN(X, Y) ((X) <= (Y) ? (X) : (Y))
#define INRANGE(X, Y, Z) \
	((((X) <= (Y)) && ((Y) <= (Z))) || \
	 (((Z) <= (Y)) && ((Y) <= (X))) ? 1 : 0)

#define MAKEWORD(X, Y) (((X) << 8) + (Y))

#define LSB(X) (((X) & 0xff))
#define MSB(Y) (((Y) >> 8) & 0xff)
#define MMSB(Y)(((Y) >> 16) & 0xff)

#define HHM_FAILURE                 ( -1 )
#define XXX 1
typedef unsigned char uint8_t ;

typedef struct inspur_i2c
{
	uint8_t BusNo;		/* SMBus Number, start from 0				*/
	uint8_t SlaveAddr;	/* SMBus slave address, higher 7-bits		*/
	uint8_t reg_low;
	uint8_t reg_high;
	uint8_t flag;
	unsigned short len;
	int fd;

} SMBusType;

#ifdef XXX
ssize_t inspur_i2c_read( SMBusType *bus,  char *data)
{
	ssize_t ret;

	if( ioctl( bus->fd, I2C_SLAVE, bus->SlaveAddr) < 0 )
	{
		printf( "Cannot set remote slave device for master write" );
		return( HHM_FAILURE );
	}


	/* Read the specified data onto the bus */
	ret = read( bus->fd, data, bus->len);
	if( (size_t)ret != bus->len)
	{
		errno = EREMOTEIO;
	}


	return ret;

}

ssize_t inspur_i2c_write( SMBusType *bus,  char *data) 
{
	ssize_t ret;

	if( ioctl( bus->fd, I2C_SLAVE, bus->SlaveAddr ) < 0 )
	{
		printf( "Cannot set remote slave device for master write" );
		return( HHM_FAILURE );
	}


	/* Write the specified data onto the bus */
	ret = write( bus->fd, data, bus->len);
	if( (size_t)ret != bus->len)
	{
		errno = EREMOTEIO;
	}


	return ret;

}
#endif


static int inspur_i2c_ioctl_write(SMBusType *bus, char *buffer)
{
	int ret = 0;
	int i = 0;
	unsigned char buff[8192] = {0};
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages;

	messages.addr = bus->SlaveAddr;
	messages.flags = bus->flag;
	messages.len = bus->len + 2;
	messages.buf = buff;
	buff[1] = bus->reg_high;
	buff[0] = bus->reg_low;

	strncpy((char *)&buff[2], buffer, bus->len);

	fprintf(stderr, "messages.buf start:\n");
	for (i = 0; i < bus->len; i++) {
		fprintf(stderr, "messages.buf[%d]:0x%x\n" ,i , messages.buf[i]);
	}
	fprintf(stderr, "messages.buf end.\n");

	packets.nmsgs = 1;
	packets.msgs = &messages;

	ret = ioctl(bus->fd, I2C_RDWR, (unsigned long)&packets);
	if (ret != packets.nmsgs){
		perror("ioctl");
		//	ret = -1;
	}

	return ret;
}
static int inspur_i2c_ioctl_read(SMBusType *bus,  char *buffer)
{

	int ret = 0;
	unsigned char buff[8192] = {0};
	struct i2c_rdwr_ioctl_data packets;
	struct i2c_msg messages;

	messages.addr = bus->SlaveAddr;
	messages.flags = I2C_M_RD;
	messages.len = bus->len + 2;
	messages.buf = buff;
	buff[1] = bus->reg_high;
	buff[0] = bus->reg_low;
	fprintf(stderr, "messages.buf[0]:0x%x, buf[1]:0x%x\n" , messages.buf[0], messages.buf[1]);


	packets.nmsgs = 1;
	packets.msgs = &messages;

	ret = ioctl(bus->fd, I2C_RDWR, (unsigned long)&packets);
	if (ret != packets.nmsgs){
		perror("ioctl");
		//	ret = -1;
	}

	strncpy(buffer, (char *)(buff+2), bus->len);
	return ret;

}
static int inspur_i2c_rdwr(SMBusType *bus,  char *buffer)
{
	char i2c_dev_name[36] = {0};
	int i2cfd;
	int ret = 0;
	sprintf(i2c_dev_name, "/dev/i2c-%d", bus->BusNo);

	i2cfd = open( i2c_dev_name, O_RDWR );
	if( i2cfd < 0 )
	{
		printf("Cannot open %s, %s\n",i2c_dev_name,strerror(errno));
		return -1;
	}

	bus->fd = i2cfd;

	if (1 == bus->flag) {
		if (bus->reg_high || bus->reg_low) 
			ret = inspur_i2c_ioctl_read(bus, buffer);
		else
			ret = inspur_i2c_read(bus, buffer);

		if (ret > 0)
			fprintf(stderr, "Read Successed\n");
		else 
			fprintf(stderr, "Read failed\n");

	} else {

		if (bus->reg_high || bus->reg_low) 
			ret = inspur_i2c_ioctl_write(bus, buffer);
		else
			ret = inspur_i2c_write(bus, buffer);

		if (ret > 0)
			fprintf(stderr, "Write Successed\n");
		else
			fprintf(stderr, "Write failed\n");
		//perror("inspur_i2c_ioctl_write");
	}

	fprintf(stderr, "ret:%d\n", ret);

	close(i2cfd);

	return ret;
}

static void help(char **argv)
{
	fprintf(stderr, "Usage:%s  i2cbus slave  reg_high reg_low rwcount [-w] [DATA]\n 2 <= i2cbus <= 8\n", argv[0]);
	exit(1);
}


int main(int argc , char *argv[])
{

	SMBusType bus;
	int busid = 0;
	long address;
	int offset1 = 0;
	int offset2 = 0;
	char *end;
	int ret = 0;
	char buffer[8190] = {0};
	int i = 0;
	int rwlen = 0;

	if (argc < 6)  
		help(argv);
	else if (argc > 6) {
		if (strncmp(argv[6], "-w", 2) != 0) help(argv);
		if (argc < 8) help(argv);
		bus.flag = 0;
	} else {

		bus.flag = 1;
	}  

	busid = strtoul(argv[1], &end, 0) ;
	if (*end || !*argv[1]) {
		fprintf(stderr, "Error: I2CBUS is not a number!\n");
		return -1;
	}
	if (busid < 2 &&  busid  > 8) 
		help(argv);
	else 
		bus.BusNo = busid;



	address = strtol(argv[2], &end, 16);

	if (*end || !*argv[2]) {
		fprintf(stderr, "Error: SLAVEADDR is not a number!\n");
		return -1;
	}

	if (address < 0x03 || address > 0x77) {
		fprintf(stderr, "Error: SLAVE address out of range "
				"(0x03-0x77)!\n");
		return -2;
	}

	bus.SlaveAddr = address;

	offset1 = strtol(argv[3], &end, 16);

	if (*end || !*argv[3]) {
		fprintf(stderr, "Error: offset1 is not a number!\n");
		return -1;
	}

	offset2 = strtol(argv[4], &end, 16);

	if (*end || !*argv[4]) {
		fprintf(stderr, "Error: offset2 is not a number!\n");
		return -1;
	}
	if (offset1 >= 255 || offset2 >= 255) fprintf(stderr, "offset2 is too long\n");

	bus.reg_high = offset1;
	bus.reg_low = offset2;

	fprintf(stderr, "slave offset :0x%x\n" , MAKEWORD(bus.reg_high, bus.reg_low));

	rwlen = strtol(argv[5], &end, 16);

	if (*end || !*argv[5]) {
		fprintf(stderr, "Error: offset2 is not a number!\n");
		return -1;
	}

	if (bus.flag == 0) {
		if (strlen(argv[7]) > 8190) {
			perror("Write data too loog, cat not greater than 8 Byte");
			return -1;
		}

		bus.len = MIN(strlen(argv[7]), rwlen);
		strncpy(buffer, argv[7], bus.len);
		fprintf(stderr,"buffer:%s\n", buffer);
		//		fprintf(stderr, "buffer[0]:0x%x\n", buffer[0]);
	} else {
		bus.len = rwlen;	
	}

	ret = inspur_i2c_rdwr(&bus, buffer);

	if (ret > 0) {
		if ( bus.flag == 1) {
			fprintf(stderr, "read data:\n");
			for(i = 0; i < bus.len; i++)
				fprintf(stderr, "0x%x  ", buffer[i]);
			fprintf(stderr, "\n");
		}

	}

	return 0;
}
