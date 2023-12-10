/* basic testing script for midas board bringup. tests spi sensors as well as emmc chip */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>

#include "pins.h"
#include "bno_functions.h"
#include "emmc_functions.h"

#include <MS5611.h>
#include <SparkFun_Qwiic_KX13X.h>
#include <Adxl355.h>
#include <SparkFunLSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BNO08x.h>

// #define MCU_TEST
#define ENABLE_BAROMETER
// #define ENABLE_HIGHG
// #define ENABLE_LOWG
// #define ENABLE_LOWGLSM
// #define ENABLE_MAGNETOMETER
// #define ENABLE_ORIENTATION
// #define ENABLE_EMMC

#ifdef ENABLE_BAROMETER
	MS5611 MS(MS5611_CS);
#endif

#ifdef ENABLE_HIGHG
	QwiicKX134 KX;
#endif

#ifdef ENABLE_LOWG
	Adxl355 sensor(ADXL355_CS);
#endif

#ifdef ENABLE_LOWGLSM
	LSM6DS3 LSM(SPI_MODE, LSM6DS3_CS);
#endif

#ifdef ENABLE_MAGNETOMETER
	Adafruit_LIS3MDL LIS3MDL;
#endif

#ifdef ENABLE_ORIENTATION
	Adafruit_BNO08x imu;
#endif

#ifdef ENABLE_EMMC
	uint8_t buff[8192];
#endif

void setup() {
	Serial.begin(9600);

	while(!Serial);

	delay(1000);

	Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);


	Serial.println("beginning sensor test");

	#ifdef ENABLE_BAROMETER
		MS.init();
		Serial.println("barometer init successfully");
	#endif

	#ifdef ENABLE_HIGHG
		KX.beginSPI(KX134_CS, 5000000);
		if (!KX.initialize(DEFAULT_SETTINGS)) {
			Serial.println("could not init highg");
			while(1);
		}
		if(!KX.setOutputDataRate(0xb)) {
			Serial.println("could not update data rate of highg");
			while(1);
		}
		KX.setRange(3);
		Serial.println("highg init successfully");
	#endif

	#ifdef ENABLE_LOWG
		sensor.start();
		if (sensor.isDeviceRecognized()){
			sensor.initializeSensor(Adxl355::RANGE_VALUES::RANGE_2G, Adxl355::ODR_LPF::ODR_1000_AND_250);
			if (Adxl355::RANGE_VALUES::RANGE_2G != sensor.getRange()){
				Serial.println("could not set range lowg");
				while(1);
			}

			if (Adxl355::ODR_LPF::ODR_4000_AND_1000 != sensor.getOdrLpf()){
				Serial.println("could not set odrlpf lowg");
				while(1);
			}
		}
		else{
			Serial.println("could not init lowg");
			while(1);
		}
   		sensor.calibrateSensor(1);
		Serial.println("lowg init successfully");
	#endif

	#ifdef ENABLE_LOWGLSM
		if (!LSM.begin()) {
			Serial.println("could not init lowglsm");
			while(1);
		}
		Serial.println("lowglsm init successfully");
	#endif

	#ifdef ENABLE_MAGNETOMETER
		if (!LIS3MDL.begin_SPI(LIS3MDL_CS)){
			Serial.println("could not init magnetometer");
			while(1);
		}
		LIS3MDL.setOperationMode(LIS3MDL_CONTINUOUSMODE);
		LIS3MDL.setDataRate(LIS3MDL_DATARATE_5_HZ);
		LIS3MDL.setRange(LIS3MDL_RANGE_4_GAUSS);
		Serial.println("magnetometer init successfully");
	#endif

	#ifdef ENABLE_ORIENTATION
		if (!imu.begin_SPI(BNO086_CS, BNO086_INT)) {
			Serial.println("could not init orientation");
			while(1);
		}
		if (!imu.enableReport(SH2_ARVR_STABILIZED_RV, 5000)) {
			Serial.println("Could not enable stabilized remote vector");
			while(1);
		}
		Serial.println("orientation init successfully");
	#endif

	#ifdef ENABLE_EMMC
		if(!SD_MMC.setPins(EMMC_CLK, EMMC_CMD, EMMC_D0, EMMC_D1, EMMC_D2, EMMC_D3)){
			Serial.println("Pin change failed!");
			return;
		}
		// if(!SD_MMC.begin()){
		if(!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_52M, 5)){
			Serial.println("Card Mount Failed");
			return;
		}
		uint8_t cardType = SD_MMC.cardType();

		if(cardType == CARD_NONE){
			Serial.println("No SD_MMC card attached");
			return;
		}

		Serial.print("SD_MMC Card Type: ");
		if(cardType == CARD_MMC){
			Serial.println("MMC");
		} else if(cardType == CARD_SD){
			Serial.println("SDSC");
		} else if(cardType == CARD_SDHC){
			Serial.println("SDHC");
		} else {
			Serial.println("UNKNOWN");
		}

		uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
		Serial.print("SD_MMC Card Size: ");
		Serial.print(cardSize);
		Serial.println("MB");
		File f = SD_MMC.open("/midas.txt", FILE_WRITE, true);
		auto m1 = micros();
		std::fill(buff, buff + 8192, 'q');
		for(int i = 0; i < 10; i++){
			f.write(buff, 1024);
		}
		Serial.println(micros() - m1);
		f.close();
		// listDir(SD_MMC, "/", 0);
		// createDir(SD_MMC, "/mydir");
		// listDir(SD_MMC, "/", 0);
		// removeDir(SD_MMC, "/mydir");
		// listDir(SD_MMC, "/", 2);
		// writeFile(SD_MMC, "/hello.txt", "Hello ");
		// appendFile(SD_MMC, "/hello.txt", "World!\n");
		// readFile(SD_MMC, "/hello.txt");
		// deleteFile(SD_MMC, "/foo.txt");
		// renameFile(SD_MMC, "/hello.txt", "/foo.txt");
		// readFile(SD_MMC, "/foo.txt");
		// testFileIO(SD_MMC, "/test.txt");
		Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
		Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
	#endif
  
}

void loop() {

	#ifdef MCU_TEST
		Serial.println("test");
	#endif

	#ifdef ENABLE_BAROMETER
		MS.read(12);
		float pressure = static_cast<float>(MS.getPressure() * 0.01 + 26.03);
		float temperature = static_cast<float>(MS.getTemperature() * 0.01);
		float altitude = static_cast<float>(-log(pressure * 0.000987) * (temperature + 273.15) * 29.254);
		Serial.print("Pressure: ");
		Serial.print(pressure);
		Serial.print(" Temp: ");
		Serial.print(temperature);
		Serial.print(" Altitude: ");
		Serial.println(altitude);
	#endif

	#ifdef ENABLE_HIGHG
		auto data = KX.getAccelData();
		Serial.print("ax: ");
		Serial.print(data.xData);
		Serial.print(" ay: ");
		Serial.print(data.yData);
		Serial.print(" az: ");
		Serial.println(data.zData);
	#endif

	#ifdef ENABLE_LOWG
		auto data_adxl = sensor.getAccel();
		Serial.print("ax: ");
		Serial.print(data_adxl.x);
		Serial.print(" ay: ");
		Serial.print(data_adxl.y);
		Serial.print(" az: ");
		Serial.println(data_adxl.z);
	#endif

	#ifdef ENABLE_LOWGLSM
		Serial.print("gx: ");
		Serial.print(LSM.readFloatGyroX());
		Serial.print(" gy: ");
		Serial.print(LSM.readFloatGyroY());
		Serial.print(" gz: ");
		Serial.print(LSM.readFloatGyroZ());
		Serial.print(" ax: ");
		Serial.print(LSM.readFloatAccelX());
		Serial.print(" ay: ");
		Serial.print(LSM.readFloatAccelY());
		Serial.print(" az: ");
		Serial.println(LSM.readFloatAccelZ());
	#endif

	#ifdef ENABLE_MAGNETOMETER
		LIS3MDL.read();
		float mx = LIS3MDL.x_gauss;
		float my = LIS3MDL.y_gauss;
		float mz = LIS3MDL.z_gauss;
		Serial.print("mx: ");
		Serial.print(mx);
		Serial.print(" my: ");
		Serial.print(my);
		Serial.print(" mz: ");
		Serial.println(mz);
	#endif

	#ifdef ENABLE_ORIENTATION
		sh2_SensorValue_t event;
		Vec3 euler;
		if (imu.getSensorEvent(&event)) {
			switch (event.sensorId) {
				case SH2_ARVR_STABILIZED_RV:
					euler = quaternionToEulerRV(&event.un.arvrStabilizedRV, true);
				case SH2_GYRO_INTEGRATED_RV:
					euler = quaternionToEulerGI(&event.un.gyroIntegratedRV, true);
					break;
			}
			Serial.print("yaw: ");
			Serial.print(euler.y);
			Serial.print(" pitch: ");
			Serial.print(euler.z);
			Serial.print(" roll: ");
			Serial.println(euler.x);
		}
	#endif

	delay(500);
}
