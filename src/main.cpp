/* basic testing script for midas board bringup. tests spi sensors as well as emmc chip */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>
#include <MicroNMEA.h>

#include "pins.h"
#include "bno_functions.h"
#include "emmc_functions.h"
#include "TCAL9539.h"
#include "ads7138-q1.h"

#include <MS5611.h>
#include <SparkFun_Qwiic_KX13X.h>
#include <PL_ADXL355.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BNO08x.h>

// #define MCU_TEST
// #define ENABLE_BAROMETER
//#define ENABLE_HIGHG
// #define ENABLE_LOWG
// #define ENABLE_LOWGLSM
// #define ENABLE_MAGNETOMETER
// #define ENABLE_ORIENTATION
// #define ENABLE_EMMC
#define ENABLE_SD
// #define ENABLE_ADS
// #define ENABLE_GPIOEXP
// #define ENABLE_GPS
// #define ENABLE_ADV_GPIO_TEST


#ifdef ENABLE_BAROMETER
	MS5611 MS(MS5611_CS);
#endif

#ifdef ENABLE_HIGHG
	QwiicKX134 KX;
#endif

#ifdef ENABLE_LOWG
	PL::ADXL355 sensor(ADXL355_CS);
#endif

#ifdef ENABLE_LOWGLSM
	LSM6DS3Class LSM(SPI, LSM6DS3_CS, 46);
#endif

#ifdef ENABLE_MAGNETOMETER
	Adafruit_LIS3MDL LIS3MDL;
#endif

#ifdef ENABLE_ORIENTATION
	Adafruit_BNO08x imu(GpioAddress(1, 07));
#endif

#ifdef ENABLE_EMMC
	uint8_t buff[8192];
#endif

#ifdef ENABLE_SD
    #define SD_CLK 5
    #define SD_CMD 4
    #define SD_D0 6
#endif

#ifdef ENABLE_GPIOEXP

#endif

#ifdef ENABLE_GPS
char buff[32];
int idx = 0;
//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void gpsHardWareReset() {
	gpioDigitalWrite(GpioAddress(2, 017), LOW);
	delay(50);
	gpioDigitalWrite(GpioAddress(2, 017), HIGH);
	delay(2000);
}

void readI2C(char *inBuff)
{
   Wire.beginTransmission(GNSS_I2C_LOCATION);
   Wire.write((uint8_t) 0xff);
   Wire.endTransmission(false);
   Wire.requestFrom((uint8_t)GNSS_I2C_LOCATION, (uint8_t) 32);
   int i = 0;
   while (Wire.available())
   {
      inBuff[i]= Wire.read();
      i++;
   }
}

#endif

#ifdef ENABLE_ADV_GPIO_TEST
enum class State {
  WAITING,
  ACTIVATE,
  ACTIVE,
};

State currentState = State::WAITING;

int expander = 0;
int channel = 1;
int highlow = 0;
bool bypass = false;
String bypassMessage;
#endif

void setup() {
	Serial.begin(9600);

	while(!Serial);

	delay(1000);

	Serial.println("Starting SPI...");
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);


	// gpioPinMode(GpioAddress(2,013), OUTPUT);
	// gpioDigitalWrite(GpioAddress(2,013), HIGH);
	// gpioPinMode(GpioAddress(2,014), OUTPUT);
	// gpioDigitalWrite(GpioAddress(2,014), HIGH);
	// gpioPinMode(GpioAddress(2,015), OUTPUT);
	// gpioDigitalWrite(GpioAddress(2,015), HIGH);
	// gpioPinMode(GpioAddress(2,016), OUTPUT);
	// gpioDigitalWrite(GpioAddress(2,016), HIGH);

	Serial.println("beginning sensor test");

	pinMode(MS5611_CS, OUTPUT);
	pinMode(LSM6DS3_CS, OUTPUT);
	pinMode(KX134_CS, OUTPUT);
	pinMode(ADXL355_CS, OUTPUT);
	pinMode(LIS3MDL_CS, OUTPUT);
	pinMode(BNO086_CS, OUTPUT);
	pinMode(CAN_CS, OUTPUT);
	pinMode(RFM96W_CS, OUTPUT);

	digitalWrite(MS5611_CS, HIGH);
	digitalWrite(LSM6DS3_CS, HIGH);
	digitalWrite(KX134_CS, HIGH);
	digitalWrite(ADXL355_CS, HIGH);
	digitalWrite(LIS3MDL_CS, HIGH);
	digitalWrite(BNO086_CS, HIGH);
	digitalWrite(CAN_CS, HIGH);
	digitalWrite(RFM96W_CS, HIGH);

	// delay(1);
	// digitalWrite(KX134_CS, HIGH);
	// delay(1);
	// digitalWrite(KX134_CS, LOW);

	#ifdef ENABLE_BAROMETER
		MS.init();
		Serial.println("barometer init successfully");
	#endif

	#ifdef ENABLE_HIGHG
		KX.beginSPI(KX134_CS, 100000);
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
		Serial.println("Before sensor.start");
		sensor.begin();
		Serial.println("Initializing lowg");
		sensor.enableMeasurement();

		// if (sensor.isDeviceRecognized()){
		// 	Serial.println("Device is recognized");
		// 	sensor.initializeSensor(Adxl355::RANGE_VALUES::RANGE_2G, Adxl355::ODR_LPF::ODR_1000_AND_250);
		// 	Serial.println("Sensor is initialized");
		// 	if (Adxl355::RANGE_VALUES::RANGE_2G != sensor.getRange()){
		// 		Serial.println("could not set range lowg");
		// 		while(1);
		// 	}

		// 	if (Adxl355::ODR_LPF::ODR_4000_AND_1000 != sensor.getOdrLpf()){
		// 		Serial.println("could not set odrlpf lowg");
		// 		while(1);
		// 	}
		// }
		// else{
		// 	Serial.println("could not init lowg");
		// 	while(1);
		// }
   		// sensor.calibrateSensor(1);
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

		/*if (!TCAL9539Init()) {
			Serial.println("Failed to initialize TCAL9539!");
			// while(1){ };
		}

		Serial.println("TCAL9539 initialized successfully!");*/
		Serial.println("Delaying");
		delay(5000);

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
		if(!SD_MMC.begin("/sdcard", false, false, SDMMC_FREQ_52M, 5)){
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
//		File f = SD_MMC.open("/midas.txt", FILE_WRITE, true);
//		auto m1 = micros();
//		std::fill(buff, buff + 8192, 'q');
//		for(int i = 0; i < 10; i++){
//			f.write(buff, 8192);
//		}
//		Serial.println(micros() - m1);
//		f.close();
		// listDir(SD_MMC, "/", 0);
		// createDir(SD_MMC, "/mydir");
		// listDir(SD_MMC, "/", 0);
		// removeDir(SD_MMC, "/mydir");
		 listDir(SD_MMC, "/", 2);
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

    #ifdef ENABLE_SD
        if(!SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0)){
            Serial.println("Pin change failed!");
            return;
        }
        // if(!SD_MMC.begin()){
        if(!SD_MMC.begin("/sd", true, true, SDMMC_FREQ_52M, 5)){
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
//		File f = SD_MMC.open("/midas.txt", FILE_WRITE, true);
//		auto m1 = micros();
//		std::fill(buff, buff + 8192, 'q');
//		for(int i = 0; i < 10; i++){
//			f.write(buff, 8192);
//		}
//		Serial.println(micros() - m1);
//		f.close();
        listDir(SD_MMC, "/", 0);
        createDir(SD_MMC, "/mydir");
        listDir(SD_MMC, "/", 0);
        removeDir(SD_MMC, "/mydir");
        listDir(SD_MMC, "/", 2);
        writeFile(SD_MMC, "/hello.txt", "Hello ");
        appendFile(SD_MMC, "/hello.txt", "World!\n");
        readFile(SD_MMC, "/hello.txt");
        deleteFile(SD_MMC, "/foo.txt");
        renameFile(SD_MMC, "/hello.txt", "/foo.txt");
        readFile(SD_MMC, "/foo.txt");
        testFileIO(SD_MMC, "/test.txt");
        Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
        Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));
    #endif

	#ifdef ENABLE_ADS
		if (!ADS7138Init()) {
        	Serial.println("could not init ads");
    	} else {
			Serial.println("ads init successfully");
		}
	#endif

	#ifdef ENABLE_GPIOEXP
		/*constexpr uint8_t GPIO0_ADDRESS = 0x74;
		constexpr uint8_t GPIO1_ADDRESS = 0x75;
		constexpr uint8_t GPIO2_ADDRESS = 0x77;
		constexpr uint8_t REG_OUTPUT0 = 0x2;
		constexpr uint8_t REG_OUTPUT1 = 0x3;
		uint8_t addrs[] = {GPIO0_ADDRESS, GPIO1_ADDRESS, GPIO2_ADDRESS};
		for(uint8_t addr : addrs){
			Wire.beginTransmission(GPIO0_ADDRESS);
			Wire.write(REG_OUTPUT0);
			if(!Wire.endTransmission()){
				return false;
			}
			int ct = Wire.requestFrom(GPIO0_ADDRESS, 1);
			if(ct != 1){
				return false;
			}
			int v = Wire.read();
			//REG_OUTPUT0 is set all ones on power up
			if(v != 0xff){
				return false;
			}
		}
		return true;*/

		if (!TCAL9539Init()) {
			Serial.println("Failed to initialize TCAL9539!");
			// while(1){ };
		}

		Serial.println("TCAL9539 initialized successfully!");

		gpioPinMode(GpioAddress(2, 013), OUTPUT);
		gpioPinMode(GpioAddress(2, 014), OUTPUT);
		gpioPinMode(GpioAddress(2, 015), OUTPUT);
		gpioPinMode(GpioAddress(2, 016), OUTPUT);

		Serial.println("TCAL9539 successfully set pinmode!");

	#endif


	#ifdef ENABLE_GPS
		gpioPinMode(GpioAddress(2, 017), OUTPUT);
		gpsHardWareReset();
		Wire.begin();
	#endif

	#ifdef ENABLE_ADV_GPIO_TEST
	delay(1000);
	Serial.println("--------------------------------------------------------------------------");
	Serial.println("To use, enter into terminal in the format \"expander, channel, signal\"");
	Serial.println("Valid expanders: 0-2, valid channels: 1-16, valid signals: 0/1");
	if (!TCAL9539Init()) {
		Serial.println("Failed to initialize TCAL9539!");
	}
	#endif
}

void loop() {

	#ifdef ENABLE_GPIOEXP

		gpioDigitalWrite(GpioAddress(2, 013), HIGH);
		delay(100);
		gpioDigitalWrite(GpioAddress(2, 014), HIGH);
		delay(100);
		gpioDigitalWrite(GpioAddress(2, 015), HIGH);
		delay(100);
		gpioDigitalWrite(GpioAddress(2, 016), HIGH);
		delay(100);
		Serial.println("Looped high");
		gpioDigitalWrite(GpioAddress(2, 013), LOW);
		gpioDigitalWrite(GpioAddress(2, 014), LOW);
		gpioDigitalWrite(GpioAddress(2, 015), LOW);
		gpioDigitalWrite(GpioAddress(2, 016), LOW);
		Serial.println("Looped");
		delay(500);
		
	#endif

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
		auto data_adxl = sensor.getAccelerations();
		Serial.print("ax: ");
		Serial.print(data_adxl.x);
		Serial.print(" ay: ");
		Serial.print(data_adxl.y);
		Serial.print(" az: ");
		Serial.println(data_adxl.z);
	#endif

	#ifdef ENABLE_LOWGLSM
		float ax, ay, az, gx, gy, gz;
		LSM.readAcceleration(ax, ay, az);
		LSM.readGyroscope(gx, gy, gz);

		Serial.print("gx: ");
		Serial.print(gx);
		Serial.print(" gy: ");
		Serial.print(gy);
		Serial.print(" gz: ");
		Serial.print(gz);
		Serial.print(" ax: ");
		Serial.print(ax);
		Serial.print(" ay: ");
		Serial.print(ay);
		Serial.print(" az: ");
		Serial.println(az);
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

	#ifdef ENABLE_ADS
		for (int i = 0; i < 8; i++) {
			Serial.print("Address ");
			Serial.print(i);
			Serial.print(": ");
			Serial.print(adcAnalogRead(ADCAddress{i}).value + ", ");
		}
		Serial.println();
	#endif

	#ifdef ENABLE_GPS
	char c ;
      if (idx == 0)
      {
         readI2C(buff);
         delay(1);
      }
      //Fetch the character one by one
      c = buff[idx];
      idx++;
      idx %= 32;
      //If we have a valid character pass it to the library
      if ((uint8_t) c != 0xFF)
      {
         Serial.print(c);
         nmea.process(c);
      }
	 // Serial.print("Valid fix: ");
     // Serial.println(nmea.isValid() ? "yes" : "no");
	#endif

	#ifdef ENABLE_ADV_GPIO_TEST // serial monitor format: "expander, channel, signal", only accepts this format, unknown behavior otherwise (todo)
	switch (currentState) {
		case State::WAITING:
			if (Serial.available() > 0 || bypass) {
				String message;
				if (bypass) {
					message = bypassMessage;
				} else {
					message = Serial.readStringUntil('\n');
				}
				int numbers[3];
				// this for loop strictly only parses format "num, num, num"
				for (int i = 0; i < 2; i++) {
					int commaIndex = message.indexOf(",");
					String numberString = message.substring(0, commaIndex);
					numbers[i] = numberString.toInt();
					message.remove(0, commaIndex + 2);
				}
				String numberString = message.substring(0, message.length());
				numbers[2] = numberString.toInt();
				message.remove(0, message.length());
				expander = numbers[0];
				channel = numbers[1];
				highlow = numbers[2];
				Serial.println("expander: " + String(expander) + ", channel: " + String(channel) + ", signal: " + String(highlow));
				currentState = State::ACTIVATE;
				break;
			}
			break;
		case State::ACTIVATE:
			gpioPinMode(GpioAddress(expander, channel), OUTPUT);
			delay(100);
			gpioDigitalWrite(GpioAddress(expander, channel), highlow);
			currentState = State::ACTIVE;
		case State::ACTIVE:
			if (Serial.available() > 0) {
				String message = Serial.readStringUntil('\n');
				if (message.equals("WAIT")) {
					Serial.println("State: WAIT");
					currentState = State::WAITING;
					bypass = false;
					//   gpioDigitalWrite(GpioAddress(expander, channel), 0); // uncomment to reset channel back to zero after use
					break;
				} else {
					currentState = State::WAITING;
					bypass = true;
					bypassMessage = message;
					//   gpioDigitalWrite(GpioAddress(expander, channel), 0); // uncomment to reset channel back to zero after use
				}
			}
			break;
	}
	#endif
	delay(500);
}
// 1, 0, 1
