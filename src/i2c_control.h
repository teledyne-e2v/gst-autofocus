#pragma once

#include "i2c.h"

int i2cInit(I2CDevice *device, I2CDevice *devicepda, int *bus);
void initDevice(I2CDevice *device, int bus, int addr, int pageByte, int iaddrBytes);
int enable_VdacPda(I2CDevice device, int bus);
int disable_VdacPda(I2CDevice device, int bus);
int write_VdacPda(I2CDevice device, int bus, int PdaRegValue);
int testPattern(I2CDevice device, int bus);
