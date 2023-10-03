#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

#include <WiFiClientSecure.h>
#include <time.h>

#include "I2Cdev.h"
#include <MPU6050_6Axis_MotionApps_V6_12.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


#include <MQTT.h>
// Include My Own File That Has Standard class for reading From Gyro 
//#define T // testing stuff like the button instead of the fsrs;
//#define D // serial.printing;
//#define DT // serial.printing data;
//#define DG // serial.printing detailled debugging stuff;
#define PERIOD 10
#define INSOLER  // INSOLEL: For Left, INSOLER: For Right

#include "secrets.h"
#include "Essentials.h"

WiFiClientSecure net;
MQTTClient client(2048);

S_Mpu mpu;

// Analog pins
const int analogIns = 5;
const int analogPins[analogIns] = { 39,34,35,33,32 };// , A4, A5, A6, A7, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19};
const char* ntpServer = "time.cloudflare.com";

static bool mpu_good = false;

bool blinkState = false;

unsigned long lastMillis = 0;

// Function to connect to Wi-Fi
inline void connectWiFi(bool skip = true) {
	if (skip && WiFi.isConnected() == WL_CONNECTED)
		return;
#ifdef D
	Serial.print("Connecting to ");
	Serial.print(ssid);
	Serial.print(" ..");
#endif
	if (!skip) {
		WiFi.setSleep(false);
		WiFi.useStaticBuffers(true);
		WiFi.setAutoConnect(true);
		WiFi.setAutoReconnect(true);
		WiFi.mode(WIFI_STA);
	}
	while (WiFi.status() != WL_CONNECTED) {
		WiFi.disconnect();
		WiFi.scanNetworks();
		WiFi.begin(ssid, password);
		for (int i = 0; i < 15; i++) {
#ifdef D
			Serial.print(".");
#endif	
			if (WiFi.status() == WL_CONNECTED)
				break;
			delay(500);
		}
		delay(1000);
#ifdef D
		Serial.println("loopy");
#endif
	}
	return;
}

//Function To Set Up Time
inline void setupRTC() {
#ifdef D
	Serial.print("[RTC] Checking wifi..");
#endif
	connectWiFi();
#ifdef D
	Serial.print("Setting Up NTP using The Server: ");
	Serial.print(ntpServer);
	Serial.print(" ..");
#endif
	configTime(0, 0, ntpServer);
	delay(10);
	
	struct tm timeinfo;

	while (!getLocalTime(&timeinfo)) {
		delay(500);
#ifdef D
		Serial.print(".");
#endif
	}

#ifdef D
	Serial.print("\nNTP Server Have Been Set, Current Time is: ");
	Serial.print(timeinfo.tm_year);
	Serial.print("/");
	Serial.print(timeinfo.tm_mon);
	Serial.print("/");
	Serial.print(timeinfo.tm_mday);
	Serial.print(" ");
	Serial.print(timeinfo.tm_hour);
	Serial.print(":");
	Serial.print(timeinfo.tm_min);
	Serial.println("\n");
#endif
}

// Function to connect to MQTT broker
inline void connectMQTT(bool skip = true) {
#ifdef D
	Serial.println("[MQTT] Checking wifi..");
#endif  
	connectWiFi();

#ifdef D
	Serial.println("\nConnecting to MQTT Broker..");
#endif 
	if (!skip) {
		net.setCACert(AWS_CERT_CA);
		net.setCertificate(AWS_CERT_CRT);
		net.setPrivateKey(AWS_CERT_PRIVATE);
#ifdef D
		Serial.print("\nAws Cert Is Set up!!!");;
#endif   
		// MQTT brokers usually use port 8883 for secure connections.
		client.begin(mqttBroker, mqttPort, net);

		// REceive Messages
		//client.onMessage(messageReceived);
	}
#ifdef INSOLER
	while (!client.connect("InsoleR-0x5c1a7ec1", skip)) {
#endif
#ifdef INSOLEL
	while (!client.connect("InsoleL-0x5c1a7ec1", skip)) {
#endif
	
		Serial.print(".");
		delay(500);
	}

#ifdef D
	Serial.println("\nMQTT Broker Connected!");
#endif

	// Subs:
	// client.subscribe("/hello");
	// client.unsubscribe("/hello");
}

// Function to connect to MQTT broker
inline void connectMPU(bool skip = true) {
	mpu_good = mpu.verify();
	if (skip && mpu_good) {
		return;
	}
#ifdef D
	Serial.println("Connecting to MPU6050 ...");
#endif
	bool failed = !mpu.begin(skip);
	if (failed) {
#ifdef D
		Serial.println("");
		Serial.print("Failed to find MPU6050 chip, Retrying ..");
#endif
	}
	else {
		mpu_good = true;
		return;
	}

	while (!mpu.begin(skip)) {
		delay(1000);
#ifdef D
		Serial.print(".");
#endif
	}

	mpu_good = true;
	return;
}

// Functions To Create Text From Data
inline void mpu2str(char* buffer200, Val_MPU data) {
	sprintf(buffer200, "{\"Type\":\"MPU\", \"Sx\": %.4f, \"Sy\": %.4f, \"Sz\": %.4f, \"Qx\": %.4f, \"Qy\": %.4f, \"Qz\": %.4f, \"Qw\": %.4f}", data.Sx, data.Sy, data.Sz, data.Qx, data.Qy, data.Qz, data.Qw);
	// {"Type":"MPU", "Sx": 10000.0000,"Sy": 10000.0000,"Sz": 10000.0000,"Qx": 10000.0000,"Qy": 10000.0000,"Qz": 10000.0000,"Qw": 10000.0000}
	return;
}

inline void analog2str(char* buffer300, uint16_t data[analogIns]) {
	char val[50];
	//strcpy(buffer300, "{\"Type\":\"PRS\"");
	//for (int i = 0; i < analogIns; i++) {
	//	sprintf(val, "\", A%u\": %u", i, data[i]);
	//	strcat(buffer300, val);
	//}
	//strcat(buffer300, "}");
	// {"Type":"PRS", "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000, "A0": 10000}

	strcpy(buffer300, "{\"Type\":\"PRS\", \"Data\": [");
	for (int i = 0; i < analogIns; i++) {
		if (i == 0)
			sprintf(val, " %u", data[i]);
		else
			sprintf(val, ", %u", data[i]);
		//Serial.print(val);
		strcat(buffer300, val);
	}
	strcat(buffer300, "]}");
	// {"Type":"PRS", "Data": [ 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000]}

	return;
}

inline void data2str(char* buffer500, Val_MPU mdata, uint16_t pdata[analogIns], struct timeval tv_now) {
	// {"Timestamp": 1_695_483_628_938_862, "MPU": {"Sx": 10000.0000,"Sy": 10000.0000,"Sz": 10000.0000,"Qx": 10000.0000,"Qy": 10000.0000,"Qz": 10000.0000,"Qw": 10000.0000}, "PSR": [ 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000, 10000]}
	char val[50];
	sprintf(buffer500, "{\"Timestamp\": %lu%06lu, \"MPU\": {\"Sx\": %.4f, \"Sy\": %.4f, \"Sz\": %.4f, \"Qx\": %.4f, \"Qy\": %.4f, \"Qz\": %.4f, \"Qw\": %.4f}, \"PSR\": [",(uint32_t)tv_now.tv_sec, (uint32_t)tv_now.tv_usec, mdata.Sx, mdata.Sy, mdata.Sz, mdata.Qx, mdata.Qy, mdata.Qz, mdata.Qw);
	for (int i = 0; i < analogIns; i++) {
		if (i == 0)
			sprintf(val, " %u", pdata[i]);
		else
			sprintf(val, ", %u", pdata[i]);
		//Serial.print(val);
		strcat(buffer500, val);
	}
	strcat(buffer500, "]}");

	return;
}

// Function to publish sensor data
inline void publishSensorData(Val_MPU data) {
	// InsoleR-MPU-0x5c1a7ec1
	char payload[200];
	mpu2str(payload, data);
	//Serial.print("Message is: ");
	//Serial.println(payload);
	//Serial.print('.');
#ifdef INSOLER
	client.publish("InsoleR-MPU-0x5c1a7ec1", payload);
#endif
#ifdef INSOLEL
	client.publish("InsoleL-MPU-0x5c1a7ec1", payload);
#endif
}

inline void publishSensorData(uint16_t data[analogIns]) {
	// InsoleR-PSR-0x5c1a7ec1
	char payload[300];
	analog2str(payload, data);
	//Serial.print("Payload is:");
	//Serial.println(payload);
	//Serial.print('.');
#ifdef INSOLER
	client.publish("InsoleR-PSR-0x5c1a7ec1", payload);
#endif
#ifdef INSOLEL
	client.publish("InsoleL-PSR-0x5c1a7ec1", payload);
#endif
}

inline void publishSensorData(Val_MPU mdata, uint16_t pdata[analogIns], struct timeval tv_now) {
	// InsoleR-MPU-0x5c1a7ec1
	char payload[500];
	data2str(payload, mdata, pdata, tv_now);
	//Serial.print("Message is: ");
	//Serial.println(payload);
	//Serial.print('.');
#ifdef INSOLER
	client.publish("InsoleR-0x5c1a7ec1", payload);
#endif
#ifdef INSOLEL
	client.publish("InsoleL-0x5c1a7ec1", payload);
#endif
}

//Function to set Pin Mode On Startup
inline void setAnalogPins() {
	for (int i = 0; i < analogIns; i++) {
		pinMode(analogPins[i], INPUT);
	}
}
// Function to read analog values
inline void readAnalogValue(uint16_t *buff) {
	for (int i = 0; i < analogIns; i++) {
		buff[i] = analogRead(analogPins[i]);
	}
	return;
}

double err_q = 0.0;
int err_q_count = 0;
double fixed = 0;

Val_MPU mpu_data_last, mpu_data;

//void messageReceived(String& topic, String& payload) {
//    Serial.println("incoming: " + topic + " - " + payload);
//
//    // Note: Do not use the client in the callback to publish, subscribe or
//    // unsubscribe as it may cause deadlocks when other things arrive while
//    // sending and receiving acknowledgments. Instead, change a global variable,
//    // or push to a queue and handle it in the loop after calling `client.loop()`.
//}

void setup() {
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
	setAnalogPins();
#ifdef T
	pinMode(23, INPUT_PULLDOWN);
#endif	

#ifdef D
	while (!Serial_Init(115200)) { delay(10); }
#endif

	// We start by connecting to a WiFi network
#ifdef D
	Serial.println();
	Serial.println("***************************************");
	Serial.println("Starting...");
	Serial.println("");
#endif
	// Connecting to WIFI
	connectWiFi(false);

#ifdef D
	Serial.println("");
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	delay(1000);
	Serial.println(WiFi.localIP());
#endif

	// Setting The Time Up
	setupRTC();

	connectMQTT(false);

	// Try to initialize MPU
	connectMPU(false);

#ifdef D
	Serial.println("");
	Serial.println("MPU6050 Found!");
#endif

	digitalWrite(2, 1);
}

uint64_t i = 0;

uint16_t analogValues[analogIns] = { 0,0,0,0,0 };

void loop() {
#ifdef D
#ifdef DG
	Serial.print(i);
	Serial.println("# : ------");
	Serial.println("Mqtt Loop!");
#endif
#endif
	client.loop();
	delay(5);  // <- fixes some issues with WiFi stability
	// publish a message roughly every Period ms.
#ifdef D
#ifdef DG
	Serial.println("Waiting For Period");
#endif
#endif
	while (millis() - lastMillis < PERIOD) {
		//mpu.resetFIFO();
		delay(1);
	}
	lastMillis = millis();
	i++;
#ifdef D
#ifdef DG
	Serial.println("Checking For Broken Stuff");
#endif
#endif
	// Check if Mqtt Connection broke
	if (!client.connected()) {
#ifdef D
		Serial.println("MQTT Connection Broke! Reconnecting ...");
#endif
		connectMQTT();
	}
	// Check if Wi-Fi is still connected
	if (WiFi.status() != WL_CONNECTED) {
#ifdef D
		Serial.println("Wifi Is Broken!");
#endif
		connectWiFi();
	}


	if (i >= 500) {
#ifdef D
#ifdef DG
		Serial.println("Checking the MPU");
#endif
#endif
		i = 0;
		if (!(mpu_good = mpu.verify())) {
#ifdef D
			Serial.println("MPU Broken!");
#endif
			connectMPU();
		}
	}
	// Check if MQTT connection is still alive
	//if (!mqttClient.connected()) {
	/*if (!mqttClient.connected()) {
#ifdef D
				Serial.print("MQTT Not Connnected!!! It Took: ");
				Serial.print(millis()-tt);
				Serial.println(" ms ;");
#endif

				connectMQTT();
			}*/

			// Check if MPU connection is still alive
			//if (!mpu_data.Sx && !mpu_data.Sy && !mpu_data.Sz && !mpu_data.Qx && !mpu_data.Qy && !mpu_data.Qz && 36.53 == mpu_data.Qw) {
			//	if (!susMPU)
			//		susMPU = true;
			//	else
			//		// Try to Reinitialize The MPU
			//		connectMPU();
			//}

#ifdef D
#ifdef DG
	Serial.println("Reading Anlg!");
#endif
#endif


	// Read analog values
	readAnalogValue(analogValues);

	//TODO: calculate "threshold" as atleast on fsr > 0.4v or two with sum > 0.7 or average > 0.2;
	//TODO: average 10 or 5 threasholds to be sure it is fixed
	//TODO: fixed should also be set if gyro data is very close to zero for a 100 readings~
	//TODO: add a moving average offset to the acceleration that moves if fixed.
	//TODONE: Used Quaterion difference to be sure if speed is fixed


	//Get TimeStamp
	struct timeval tv_now;
	gettimeofday(&tv_now, NULL);

	// Read MPU6050 sensor data
#ifdef T
	fixed = digitalRead(23);
#endif
#ifdef D
#ifdef DG
	Serial.println("Reading MPU!");
#endif
#endif
	mpu_data = mpu.read(fixed);
	int tt = millis();
	while (mpu_data == (Val_MPU) { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }) {
		if (millis() - tt > 200) {
			//if (!(mpu_good = mpu.verify())) {
				connectMPU();
			//}
		}
		delay(1);
		mpu_data = mpu.read();
	}
#ifdef D
#ifdef DG
	Serial.println("Mpu Read!  Correcting the data!");
#endif
#endif
	err_q = abs(mpu_data.Qx - mpu_data_last.Qx);
	err_q += abs(mpu_data.Qy - mpu_data_last.Qy);
	err_q += abs(mpu_data.Qz - mpu_data_last.Qz);
	err_q += abs(mpu_data.Qw - mpu_data_last.Qw);

	if (err_q < 0.0012)
		err_q_count++;
	else {
		fixed = 0;
		err_q_count = 0;
	}


	if (err_q_count >= 20) {
		fixed += 0.02;
		fixed = min(fixed, 1.0);
		err_q_count = 0;
	}
	//mpu_data.Sz = fixed;
#ifdef D
#ifdef DG
	Serial.println("Publishing Data");
#endif
#endif
	// Publish sensor data
	if (!is_odd(i)) 
		publishSensorData(mpu_data, analogValues,tv_now);
	//publishSensorData(mpu_data);
	//publishSensorData(analogValues);

	// Save The MPU Data to be used again
	mpu_data_last = mpu_data;
#ifdef D
#ifdef DG
	Serial.println("Toggeling Led Pin");
#endif
#endif
	if(i&0b100 && i & 0b10 && i&0b1)
		digitalWrite(LED_PIN, blinkState = !blinkState);

}
