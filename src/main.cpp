/* basic testing script for midas board bringup. tests spi sensors as well as emmc chip */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <FS.h>
#include <SD_MMC.h>
#include <MicroNMEA.h>
#include <RH_RF95.h>

#include "pins.h"
#include "bno_functions.h"
#include "emmc_functions.h"
#include "TCAL9539.h"
#include "teseo_liv3f_class.h"
#include "ads7138-q1.h"

#include <MS5611.h>
#include <SparkFun_Qwiic_KX13X.h>
#include <PL_ADXL355.h>
#include <Arduino_LSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_BNO08x.h>

#include <ACAN2517FD.h>
#include <ACAN2517FDSettings.h>

// #define MCU_TEST
// #define ENABLE_BAROMETER
// #define ENABLE_HIGHG
// #define ENABLE_LOWG
// #define ENABLE_LOWGLSM
// #define ENABLE_MAGNETOMETER
// #define ENABLE_ORIENTATION
// #define ENABLE_EMMC
// #define ENABLE_ADS
// #define ENABLE_GPIOEXP
// #define ENABLE_GPS
// #define ENABLE_TElEMETRY
// #define ENABLE_ADV_GPIO_TEST
// #define ENABLE_CAN
#define ENABLE_PWR_MONITOR


#ifdef ENABLE_CAN
	ACAN2517FD can (CAN_CS, SPI, 0) ; // You can use SPI2, SPI3, if provided by your microcontroller
	static unsigned gSendDate = 0 ;
	static unsigned gSentCount = 0 ;
#endif

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

#ifdef ENABLE_GPIOEXP

#endif

#ifdef ENABLE_GPS
TeseoLIV3F teseo(&Wire, GPS_RESET, GPS_ENABLE);
#endif


#ifdef ENABLE_TElEMETRY
	 RH_RF95 rf95 (TELEMETRY_CS,TELEMETRY_INT);
#endif

#ifdef CAN_ENABLE
	// CAN can be enabled here
	ACAN2517FD can (CAN_CS, SPI, CAN_INT);
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
    SPI.begin(SPI_SCK, SPI_MOSI, SPI_MISO);

    Serial.println("Starting I2C...");
    Wire.begin(I2C_SDA, I2C_SCL);

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

	#ifdef ENABLE_CAN

		// I like having flashing lights.
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


      	Serial.println("MIDAS: Initializing CAN Controller");

		pinMode(CAN_CS, OUTPUT);
		gpioPinMode(GpioAddress(2, CAN_INT), INPUT);
		gpioPinMode(GpioAddress(2, CAN_FAULT), INPUT);
		// pinMode(CAN_SILENT, OUTPUT);

		ACAN2517FDSettings can_settings (ACAN2517FDSettings::OSC_40MHz, 125*1000, DataBitRateFactor::x1  );
		can_settings.mRequestedMode = ACAN2517FDSettings::InternalLoopBack;
		

		const uint32_t errorCode = can.begin (can_settings, [] { can.isr () ; }) ;
		if (0 == errorCode) {
			Serial.println ("MIDAS: Can ok") ;
		}else{
			Serial.print ("MIDAS: Error Can: 0x") ;
			Serial.println (errorCode, HEX) ;
		}


	#endif

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

		Serial.println("Delaying");
		delay(1000);

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

		if(!SD_MMC.begin("/sdcard", false, true, SDMMC_FREQ_52M, 5)){
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
			f.write(buff, 8192);
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
	Serial.println("Initing GPS");
	if (!teseo.init()) {
		Serial.println("Failed to init GPS");
	} else {
		Serial.println("Successfully inited GPS");
	}
	#endif

	#ifdef ENABLE_TElEMETRY
		pinMode(TELEMETRY_RESET, OUTPUT); //reset pin for RFM96W
		digitalWrite(TELEMETRY_RESET, HIGH);
		delay(100);
		digitalWrite(TELEMETRY_RESET, LOW);
		delay(100);
		digitalWrite(TELEMETRY_RESET, HIGH);
		delay(5);
		

		//rf95.init();
		Serial.println(rf95.init());
		rf95.setFrequency(433.0);
		rf95.setTxPower(23, false);
		sei();
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

int read_reg(int reg, int bytes) {
    Wire.beginTransmission(0x44);
    Wire.write(reg);
    if(Wire.endTransmission()){
        Serial.println("I2C Error");
    }
    Wire.requestFrom(0x40, bytes);
    int val = 0;
    for(int i = 0; i < bytes; i++){
        int v = Wire.read();
        if(v == -1) Serial.println("I2C Read Error");
        val = (val << 8) | v;
    }
    return val;
}


void loop() {

	#ifdef ENABLE_PWR_MONITOR
		int power = read_reg(0x8, 3);
		int current = read_reg(0x7, 2);
		int temp = read_reg(0x6, 2);
		int voltage = read_reg(0x5, 2);
		Serial.print("Voltage ");
		Serial.println(voltage * 3.125 / 1000.0);
		Serial.print("Temp ");
		Serial.println(temp * 125 / 1000.0);
		Serial.print("Current ");
		Serial.println(current * 1.2 / 1000.0);
		Serial.print("Power ");
		Serial.println(power * 240 / 1000000.0);
	#endif

    #ifdef ENABLE_CAN
      delay(500);

      Serial.println("Controller phase:");
      

      CANFDMessage message ;
      if (gSendDate < millis ()) {

        message.id = 0x542;

        message.len = 8;
        
        message.data[0] = 0x01;
        message.data[1] = 0x23;
        message.data[2] = 0x45;
        message.data[3] = 0x67;
        message.data[4] = 0x89;
        message.data[5] = 0xAB;
        message.data[6] = 0xCD;
        message.data[7] = 0xEF;

		Serial.println("Attempting to send message");

        const bool ok = can.tryToSend (message) ;
        if (ok) {
          gSendDate += 1000 ;
          gSentCount += 1 ;
          Serial.print ("Sent msg# ") ;
          Serial.println (gSentCount) ;
		  gpioDigitalWrite(GpioAddress(2, 014), HIGH);
		  delay(50);
		  gpioDigitalWrite(GpioAddress(2, 014), LOW);

        } else {
          Serial.println("FAILED SEND");
		  gpioDigitalWrite(GpioAddress(2, 016), HIGH);
		  delay(50);
		  gpioDigitalWrite(GpioAddress(2, 016), LOW);
        }


        // // Diagnostic info
        // Serial.println("Diag info:");
        // // Read diag register 0


        // uint32_t diag0 = can.diagInfos(0);
        // // extract and print data from diag0 in 8-bit chunks by bitshifting diag0
        // uint8_t tefl = diag0 & 0xFF;
        // uint8_t tefh = (diag0 >> 8) & 0xFF;
        // uint8_t recfl = (diag0 >> 16) & 0xFF;
        // uint8_t recfh = (diag0 >> 24) & 0xFF;

        // Serial.println(diag0, BIN);
        // Serial.print(tefl, BIN);
        // Serial.print(" ");
        // Serial.print(tefh, BIN);
        // Serial.print(" ");
        // Serial.print(recfl, BIN);
        // Serial.print(" ");
        // Serial.print(recfh, BIN);
        // Serial.print(" ");



        // Serial.printf("Diag 0: %d %d %d %d\n", tefl, tefh, recfl, recfh);



        // // Read diag register 1
        // uint32_t diag1 = can.diagInfos(1);
        // // extract and print data from diag1 in 8-bit chunks by bitshifting diag1
        // uint16_t nerr = diag1 & 0xFFFF;
        // uint16_t rest = (diag1 >> 16) & 0xFFFF;

        // Serial.printf("Diag 0: %d ", nerr);

        // Serial.println(rest, BIN);

      }

      if (can.receive (message)) {
        Serial.print ("CAN Received: ") ;
        Serial.printf("%d%d%d%d%d%d%d%d\n", message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5], message.data[6], message.data[7]);
      }

      Serial.println("Transciever phase:");

      bool can_fault = gpioDigitalRead(GpioAddress(2, CAN_FAULT)).value;
      if(can_fault) {
        Serial.println("CAN Fault");
        gpioDigitalWrite(GpioAddress(2, 016), HIGH);
		delay(1000);

      } else {
        gpioDigitalWrite(GpioAddress(2, 016), LOW);
      }


    #endif

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
			Serial.println((adcAnalogRead(ADCAddress{i}).value) * 3.3 / 4096);
		}
		Serial.println();
	#endif

	#ifdef ENABLE_GPS
	    teseo.update();
		GPGGA_Info_t gpgga_message = teseo.getGPGGAData();
		GPRMC_Info_t gprmc_message = teseo.getGPRMCData();
		GSV_Info_t gsv_message = teseo.getGSVData();
		float lat = gpgga_message.xyz.lat;
		float lon = gpgga_message.xyz.lon;
		float alt = gpgga_message.xyz.alt;
		float v = gprmc_message.speed;
		uint16_t sat_count = gpgga_message.sats;

		Serial.print("Time: ");
		Serial.print(gpgga_message.utc.hh);
		Serial.print(":");
		Serial.print(gpgga_message.utc.mm);
		Serial.print(":");
		Serial.print(gpgga_message.utc.ss);
		Serial.print(" Satellite Fixes: ");
		Serial.print(sat_count);
		Serial.print("/");
		Serial.print(gsv_message.tot_sats);
		Serial.print(" Latitude: ");
		Serial.print(lat);
		Serial.print(" Longitude: ");
		Serial.print(lon);
		Serial.print(" Altitude: ");
		Serial.print(alt);
		Serial.print(" Velocity: ");
		Serial.println(v);
	#endif

	#ifdef ENABLE_GPIOEXP
	#endif

	#ifdef ENABLE_TElEMETRY
		//Serial.println("Sending...");
		const char* payload = "Hello World!";

		// Send the payload
		while (true){
			rf95.send((uint8_t*)payload, strlen(payload));
			Serial.println("Sending...");
			rf95.waitPacketSent();
			Serial.println("Sent");
			delay(500);
		}
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
