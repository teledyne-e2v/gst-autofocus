#include "i2c_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int i2cInit(I2CDevice *device, I2CDevice *devicepda, int *bus)
{	
	int err;
	char bus_name[32] = "/dev/i2c-6"; //<--bus 6
	if ((*bus = i2c_open(bus_name)) == -1)
	{

		fprintf(stderr, "Open i2c bus:%s error!\n", bus_name);
		return -3;
	}

	printf("Bus %s open\n", bus_name);

	/* Init i2c device */
	printf("init device : \n");
	

	initDevice(device, *bus, 0x3D, 256, 1);

	/* Init i2c devicepda */
	initDevice(devicepda, *bus, 0x0C, 8, 1);

	err = enable_VdacPda(*devicepda, *bus);
	return err;
	
}

void initDevice(I2CDevice *device, int bus, int addr, int pageByte, int iaddrBytes)
{
	memset(device, 0, sizeof(*device));
	i2c_init_device(device);

	(*device).bus = bus;
	/*device address*/
	(*device).addr = addr;
	/*Unknown value*/
	(*device).page_bytes = pageByte;
	/*Address length in bytes*/
	(*device).iaddr_bytes = iaddrBytes;
}

int enable_VdacPda(I2CDevice device, int bus)
{
	int registre;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, size);

	// SET CFG_ENABLE = 1
	*buffer = 0x01;
	registre = 0x00;
	if ((i2c_ioctl_write(&device, registre, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", registre);
		i2c_close(bus);
		return(1);
	}

	return 0;
}

int disable_VdacPda(I2CDevice device, int bus)
{
	int registre;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, size);

	// SET CFG_ENABLE = 0
	*buffer = 0x00;
	registre = 0x00;
	if ((i2c_ioctl_write(&device, registre, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", registre);
		i2c_close(bus);
		return(-3);
	}

	printf("Disable i2c pda\n");

	return 0;
}

int write_VdacPda(I2CDevice device, int bus, int PdaRegValue)
{
	int registre;
	unsigned char MSB, LSB;
	unsigned char buffer[1];
	ssize_t size = sizeof(buffer);
	memset(buffer, 0, size);

	// SET DAC VALUE
	// printf("PdaRegValue=%d\n", PdaRegValue);
	if (PdaRegValue > 879)
	{
		PdaRegValue = 879;
	}
	else if (PdaRegValue < -91)
	{
		PdaRegValue = -91;
	}


	if (PdaRegValue >= 0)
	{
		MSB = 0x80 + ((PdaRegValue >> 8) & 0xFF);
		LSB = PdaRegValue & 0xFF;
	}
	else
	{
		PdaRegValue = -PdaRegValue;
		MSB = 0x0 + ((PdaRegValue >> 8) & 0xFF);
		LSB = PdaRegValue & 0xFF;
	}
	// printf("Val=%d, MSB=0x%x, LSB=0x%x\n", PdaRegValue, MSB, LSB);
	device.page_bytes = 8;
	// WRITE MSB
	registre = 0x02;
	*buffer = MSB;
	if ((i2c_ioctl_write(&device, registre, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", registre);
		i2c_close(bus);
		return(-3);
	}
	// dumpPda50Reg(device, bus);
	//  WRITE LSB
	registre = 0x03;
	*buffer = LSB;
	if ((i2c_ioctl_write(&device, registre, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", registre);
		i2c_close(bus);
		return(-3);
	}
	// dumpPda50Reg(device, bus);
	return 0;
}

/************************************************
*Test Pattern ON/OFF

Config: 0x38=0x013B
init val=0
inc column=3
inc row=7
incr frame =4

Pattern OFF: reg_0x04=0x8008
Pattern ON: reg_0x04=0x0001

************************************************/
int testPattern(I2CDevice device, int bus)
{
	unsigned char buffer[2];
	ssize_t size = sizeof(buffer);
	int reg = 0;
	int val = 0;

	memset(buffer, 0, size);
	device.page_bytes = 256;

	// test pattern config
	reg = 0x38;
	// val=0x003B; //frame increment OFF
	val = 0x00; // frame increment ON
	*buffer = ((val)&0xff00) >> 8;
	*(buffer + 1) = ((val)&0x00ff);
	if ((i2c_ioctl_write(&device, reg, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", reg);
		i2c_close(bus);
		return(-3);
	}

	// read test pattern status
	reg = 0x04;
	val = 0;
	memset(buffer, 0, size);
	if ((i2c_ioctl_read(&device, reg, buffer, size)) != size)
	{
		fprintf(stderr, "Can't read in %x reg!\n", reg);
		i2c_close(bus);
		return(-3);
	}
	else
	{
		val = (*buffer << 8) + *(buffer + 1);
	}

	// test pattern toggle ON/OFF
	if (val == 0x8008)
		val = 0x0001;
	else
		val = 0x8008;

	reg = 0x04;
	*buffer = ((val)&0xff00) >> 8;
	*(buffer + 1) = ((val)&0x00ff);
	if ((i2c_ioctl_write(&device, reg, buffer, size)) != size)
	{
		fprintf(stderr, "Can't write in %x reg!\n", reg);
		i2c_close(bus);
		return(-3);
	}

	return 0;
}
