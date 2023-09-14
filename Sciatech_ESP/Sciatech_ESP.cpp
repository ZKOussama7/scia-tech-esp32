#include <Arduino.h>

#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiSTA.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Include Libraries for accelerometer readings from Adafruit MPU6050
#include "I2Cdev.h"
#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

// Include Mqtt Library 
#include <PubSubClient.h>


// Include My Own File That Has Standard class for reading From Gyro 
//#include "MPU6050_TOCKN.h"
#include "Essentials.h"

// Analog pins
const int analogIns = 5;
const int analogPins[analogIns] = { 39,34,35,33,32};// , A4, A5, A6, A7, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19};


const char* ssid = "ORANGE-DIGITAL-CENTER"; // Change this to your WiFi SSID
const char* password = "Welcome@2021"; // Change this to your WiFi password

// MQTT broker details
const char* mqttBroker = "test.mosquitto.org";
const int mqttPort = 1883;

static bool susMPU = false;

S_Mpu mpu;
WiFiClient wifi_client;

// MQTT client
PubSubClient mqttClient(wifi_client);

// Function to connect to Wi-Fi
inline void connectWiFi(char* ssid, char* password) {
	Serial.print("Connecting to ");
	Serial.print(ssid);
	Serial.print(" ..");

	while (!Wifi_Init(ssid, password)) {
		for (int i = 0; i < 12 && !WiFi.isConnected(); i++) {
			Serial.print(".");
			delay(500);
		}
		if (WiFi.isConnected())
			break;

		delay(1000);
		WiFi.disconnect();
		wifi_began = false;
		Serial.println("loopy");
	}
}

// Function to connect to MQTT broker
inline void connectMQTT() {
	while (!mqttClient.connected()) {
		Serial.print("Connecting to MQTT Broker...");
		String clientId = "ESP32SCIATECH-";
		clientId += String(millis(), HEX);
		if (mqttClient.connect(clientId.c_str())) {
			Serial.println("Connected");
		}
		else {
			Serial.print("Failed, rc=");
			Serial.print(mqttClient.state());
			Serial.println(" Retrying in 5 seconds...");
			delay(5000);
		}
	}
}

// Function to connect to MQTT broker
inline void connectMPU() {
	Serial.println("Connecting to MPU6050 ...");

	if (!mpu.begin()) {
		Serial.println("");
		Serial.print("Failed to find MPU6050 chip, Retrying ..");
	}
	while (!mpu.begin()) {
		delay(1000);
		Serial.print(".");
	}
	susMPU = false;
	return;
}

// Functions To Create Text From Data
void mpu2str(char* buffer200, Val_MPU data) {
	sprintf(buffer200, "{\"Type\":\"MPU\", \"Ax\": %.4f, \"Ay\": %.4f, \"Az\": %.4f, \"Gx\": %.4f, \"Gy\": %.4f, \"Gz\": %.4f, \"T\": %.4f}", data.Ax, data.Ay, data.Az, data.Gx, data.Gy, data.Gz, data.T);
	// {"Type":"MPU", "Ax": 10000.0000,"Ay": 10000.0000,"Az": 10000.0000,"Gx": 10000.0000,"Gy": 10000.0000,"Gz": 10000.0000,"T": 10000.0000}
	return;
}

void analog2str(char* buffer300, uint16_t data[analogIns]) {
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

// Function to publish sensor data
void publishSensorData(Val_MPU data) {
	const char* topic_template = "ESP32SCIATECH-thisSensorNthat";
	char topic[50];
	sprintf(topic, topic_template);

	char payload[200];
	mpu2str(payload, data);
	//Serial.print("Message is: ");
	//Serial.println(payload);
	mqttClient.publish(topic, payload);
}

void publishSensorData(uint16_t data[analogIns]) {
	const char* topic_template = "ESP32SCIATECH-thisSensorNthat";
	char topic[50];
	sprintf(topic, topic_template);

	char payload[300];
	analog2str(payload, data);
	//Serial.print("Payload is:");
	//Serial.println(payload);
	mqttClient.publish(topic, payload);
}

//Function to set Pin Mode On Startup
inline void setAnalogPins() {
	for (int i = 0; i < analogIns; i++) {
		pinMode(analogPins[i], INPUT);
	}
}
// Function to read analog values
inline void readAnalogValue(uint16_t buff[analogIns]) {
	for (int i = 0; i < analogIns; i++) {
		buff[i] = analogRead(analogPins[i]);
	}
	return;
}


TaskHandle_t codeLoopTask;
TaskHandle_t mqttLoopTask;
SemaphoreHandle_t mutex;

// Gather data and publish using MQTT protocol
void codeLoop(void* pvParameters) {
	while (1) {
		// Check if Wi-Fi is still connected
		if (WiFi.status() != WL_CONNECTED) {
			connectWiFi((char*)ssid, (char*)password);
		}

		// Check if MQTT connection is still alive
		if (!mqttClient.connected()) {
			connectMQTT();
		}

		// Read MPU6050 sensor data
		Val_MPU mpu_data = mpu.read();

		// Check if MPU connection is still alive
		if (!mpu_data.Ax && !mpu_data.Ay && !mpu_data.Az && !mpu_data.Gx && !mpu_data.Gy && !mpu_data.Gz && 36.53 == mpu_data.T) {
			if (!susMPU)
				susMPU = true;
			else
				// Try to Reinitialize The MPU
				connectMPU();
		}

		// Read analog values
		uint16_t analogValues[analogIns];
		readAnalogValue(analogValues);
		// Acquire the mutex to access the shared variable
		if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
			// Publish sensor data
			publishSensorData(mpu_data);
			publishSensorData(analogValues);

			// Release the mutex
			xSemaphoreGive(mutex);
		}

	}
}

void mqttLoop(void* pvParameters) {
	// MQTT loop to send published data to the broker
	while (1) {
		// Acquire the mutex to access the shared variable
		if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE && WiFi.status() == WL_CONNECTED && mqttClient.connected() && !susMPU) {
			// use the shared mqttClient
			mqttClient.loop();
			// Release the mutex
			xSemaphoreGive(mutex);
		}
		vTaskDelay(pdMS_TO_TICKS(20)); // Adjust the delay based on your requirements
	}
}

// the setup function runs once when you press reset or power the board
void setup() {
	setAnalogPins();
	while (!Serial_Init(115200)) { delay(10); }

	// We start by connecting to a WiFi network
	Serial.println();
	Serial.println("***************************************");
	Serial.println("Starting...");
	Serial.println("");

	// Connecting to WIFI
	connectWiFi((char*)ssid, (char*)password);

	Serial.println("");
	Serial.println("WiFi connected");
	Serial.print("IP address: ");
	delay(1000);
	Serial.println(WiFi.localIP());

	// Try to initialize MPU
	connectMPU();

	Serial.println("");
	Serial.println("MPU6050 Found!");

	// Mqtt set URL
	mqttClient.setServer(mqttBroker, mqttPort);

	// Perform error handling for MQTT connection
	if (!mqttClient.connected()) {
		connectMQTT();
	}

	// Create a mutex
	mutex = xSemaphoreCreateMutex();

	// Create the codeLoop task and assign it to core 0
	xTaskCreatePinnedToCore(
		codeLoop,
		"codeLoopTask",
		16384*4,
		NULL,
		1,
		&codeLoopTask,
		0
	);

	// Create the mqttLoop task and assign it to core 1
	xTaskCreatePinnedToCore(
		mqttLoop,
		"mqttLoopTask",
		16384,
		NULL,
		1,
		&mqttLoopTask,
		1
	);
}

void loop() {
	// Other tasks or operations

	// Empty loop as tasks are running on separate cores
	vTaskSuspend(NULL);
}

#define BATCH_EVERY 15
int batch_count_down = BATCH_EVERY;

void looop() {
	// Check if Wi-Fi is still connected
	if (WiFi.status() != WL_CONNECTED) {
		connectWiFi((char*)ssid, (char*)password);
	}

	// Check if MQTT connection is still alive
	if (!mqttClient.connected()) {
		connectMQTT();
	}

	// Read MPU6050 sensor data
	Val_MPU mpu_data = mpu.read();

	// Check if MPU connection is still alive
	if (!mpu_data.Ax && !mpu_data.Ay && !mpu_data.Az && !mpu_data.Gx && !mpu_data.Gy && !mpu_data.Gz && 36.53 == mpu_data.T) {
		if (!susMPU)
			susMPU = true;
		else
			// Try to Reinitialize The MPU
			connectMPU();
	}

	// Read analog values
	uint16_t analogValues[analogIns];
	readAnalogValue(analogValues);

	// Publish sensor data
	publishSensorData(mpu_data);
	publishSensorData(analogValues);

	// Yield to allow MQTT messages to be sent
	if (batch_count_down == 0) {
		mqttClient.loop();
		batch_count_down = BATCH_EVERY;
	}
	else
		batch_count_down--;

}