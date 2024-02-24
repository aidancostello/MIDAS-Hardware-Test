#pragma once

// SPI sensor bus
#define SPI_MISO 13
#define SPI_MOSI 11
#define SPI_SCK 12

// barometer chip select
#define MS5611_CS 14

// gyro chip select
#define LSM6DS3_CS 3

// high g chip select
#define KX134_CS 10

// low g chip select
#define ADXL355_CS 0

// magnetometer chip select
#define LIS3MDL_CS 9

// orientation chip select, interrupt
#define BNO086_CS 21
#define BNO086_INT 47

// i2c bus pins
#define I2C_SDA 18
#define I2C_SCL 8

// emmc pins
#define EMMC_CLK 38
#define EMMC_CMD 39
#define EMMC_D0 44
#define EMMC_D1 43
#define EMMC_D2 2
#define EMMC_D3 42

// CAN pins
#define CAN_CS 45
#define RFM96W_CS 1