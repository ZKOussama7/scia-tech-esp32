#define LED_PIN 2
#define INTERRUPT_PIN 19 

typedef struct {
	double Sx, Sy, Sz;
	double Qx, Qy, Qz, Qw;
} Val_MPU;

bool operator==(const Val_MPU& lhs, const Val_MPU& rhs)
{
	return lhs.Sx == rhs.Sx && lhs.Sy == rhs.Sy && lhs.Sz == rhs.Sz && lhs.Qx == rhs.Qx && lhs.Qy == rhs.Qy && lhs.Qz == rhs.Qz && lhs.Qw == rhs.Qw;
}

static bool serial_began = false;
static bool wifi_began = false;

#ifdef D
bool Serial_Init(int speed) {
	if (!serial_began) {
		Serial.begin(speed);
		serial_began = true;
		delay(20);
	}
	return Serial;
}
#endif

bool Wifi_Init(const char* ssid, const char* pwd) {
	if (!wifi_began) {
		WiFi.setSleep(false);
		WiFi.useStaticBuffers(true);
		WiFi.mode(WIFI_STA);
		WiFi.disconnect();
		WiFi.scanNetworks();
		wifi_began = true;
		WiFi.begin(ssid, pwd);
		delay(10);
	}
	return WiFi.status() == WL_CONNECTED;
}

class S_Mpu
{
public:
	S_Mpu() {}

	~S_Mpu() {}

	bool begin() {
		// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
		Wire.begin();
		Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
		Fastwire::setup(400, true);
#endif

		// initialize device
#ifdef D
		Serial.println("Initializing I2C devices...");
#endif
		_mpu.initialize();
		pinMode(INTERRUPT_PIN, INPUT);

		if (!verify()) return false;

#ifdef D
		Serial.println(F("Initializing DMP..."));
#endif
		devStatus = _mpu.dmpInitialize();

		if (devStatus != 0) {
			// ERROR!
			// 1 = initial memory load failed
			// 2 = DMP configuration updates failed
			// (if it's going to break, usually the code will be 1)
#ifdef D
			Serial.print(F("DMP Initialization failed (code "));
			Serial.print(devStatus);
			Serial.println(F(")"));
#endif
			return false;
		}

		// turn on the DMP, now that it's ready
#ifdef D
		Serial.println(F("Enabling DMP..."));
#endif 
		_mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		// Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		// Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		// Serial.println(F(")..."));
		// attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = _mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = _mpu.dmpGetFIFOPacketSize();


		// use the code below to change accel/gyro offset values
#ifdef D
		Serial.println("Updating internal sensor offsets...");
#endif
		//_mpu.CalibrateGyro(10);
		// _mpu.setXAccelOffset(-1845);
		// _mpu.setYAccelOffset(-1182);
		// _mpu.setZAccelOffset(1211);
		 //_mpu.CalibrateAccel(10);
		_mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_4);
		//_mpu.setFullScaleGyroRange(MPU6050_IMU::MPU6050_GYRO_FS_1000);
		_mpu.PrintActiveOffsets();

		// configure Arduino LED pin for output
		pinMode(LED_PIN, OUTPUT);

		return true;
	}

	bool verify() {
		// verify connection
		bool verified = _mpu.testConnection();
#ifdef D
		//Serial.println("Testing device connections...");
		//Serial.println(verified ? "MPU6050 connection successful" : "MPU6050 connection failed");
#endif
		return verified;
	}

	Val_MPU read(double fixed = 0.0) {
		Val_MPU rt = (Val_MPU){ 0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

		// if programming failed, don't try to do anything
		if (!dmpReady) return rt;
		// read a packet from FIFO
		if (!_mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return rt;
		
		_mpu.resetFIFO();

		_mpu.dmpGetQuaternion(&q, fifoBuffer);
		// q_c = q_g.getProduct(q);
		// q.normalize();
		_mpu.dmpGetAccel(&aa, fifoBuffer);
		_mpu.dmpGetGravity(&gravity, &q);
		_mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		// aaReal.x = aa.x - gravity.x*16384;
		// aaReal.y = aa.y - gravity.y*16384;
		// aaReal.z = aa.z - gravity.z*16384;
		_mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);

		rt.Qx = q.x;
		rt.Qy = q.y;
		rt.Qz = q.z;
		rt.Qw = q.w;
		spdX = spdX + ((double)aaWorld.x / 8192.0) * 0.05;
		spdX *= 1-fixed;
		spdY = spdY + ((double)aaWorld.y / 8192.0) * 0.05;
		spdY *= 1-fixed;
		spdZ = spdZ + ((double)aaWorld.z / 8192.0) * 0.05;
		spdZ *= 1-fixed;
		rt.Sx = spdX;
		rt.Sy = spdY;
		rt.Sz = spdZ;

#ifndef D
		Serial.print("quat\t");
		Serial.print(q.w);
		Serial.print("\t");
		Serial.print(q.x);
		Serial.print("\t");
		Serial.print(q.y);
		Serial.print("\t");
		Serial.print(q.z);
		Serial.print("\t");

		Serial.print("aworld\t");
		Serial.print((double)aaWorld.x / 8192.0);
		Serial.print("\t");
		Serial.print((double)aaWorld.y / 8192.0);
		Serial.print("\t");
		Serial.print((double)aaWorld.z / 8192.0);
		Serial.print("\t");
		Serial.println(" ");
#endif
		return rt;
	}

private:
	MPU6050 _mpu;
	// MPU control/status vars
	bool dmpReady = false;   // set true if DMP init was successful
	uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
	uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;      // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64];  // FIFO storage buffer

	double spdX = 0;
	double spdY = 0;
	double spdZ = 0;

	Quaternion q;         // [w, x, y, z]         quaternion container
	// Quaternion q_c;       // [w, x, y, z]         quaternion corrected
	// Quaternion q_g;       // [w, x, y, z]         initial quaternion orientation
	VectorInt16 aa;       // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
	VectorFloat gravity;  // [x, y, z]            gravity vector
	VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements

};

//class C_Raw
//{
//public:
//    C_Raw(char* url_input, int port_input, int timeout_ms=2000){
//        this->url = url_input;
//        this->port = port_input;
//        this->timeout_ms = timeout_ms;
//    }
//    C_Raw(const char* url_input, int port_input, int timeout_ms = 2000) :
//        url((char*)url_input), port(port_input), timeout_ms(timeout_ms) {}
//
//
//    ~C_Raw(){}
//
//    bool begin() {
//        if (!is_connected()) 
//            this->clt.connect(this->url, this->port, this->timeout_ms);
//        return is_connected();
//    }
//
//    bool is_connected() {
//        return this->clt.connected();
//    }
//
//    bool is_available() {
//        return this->clt.available();
//    }
//
//    bool writeLine(String Text){
//        if (!is_connected())
//            return false;
//        this->clt.println(Text);
//        return true;
//    }
//
//    String readLine(){
//        if (!is_connected())
//            return "ERROR: !connected";
//        
//        if (!is_available())
//            return "ERROR: !available";
//
//        String Line;
//        Line = clt.readStringUntil('\r');
//        return Line;
//
//        // needs to be corrected
//        //while (clt.available() == 0) {
//        //    if (millis() - timeout > 5000) {
//        //        //Serial.println(">>> Client Timeout !");
//        //        clt.stop();
//        //        return;
//        //    }
//        //}
//
//        // Read all the lines of the reply from server and print them to Serial
//       /* while (clt.available()) {
//            String line = clt.readStringUntil('\r');
//            Serial.print(line);
//        }*/
//
//        //Serial.printf("\nClosing connection\n\n");
//    }
//
//    bool waitAvailable() {
//        int timeout = millis();
//        while (!is_available()) {
//            if (millis() - timeout > this->timeout_ms) {
//                //Serial.println(">>> Client Timeout !");
//                //clt.stop();
//                return false;
//            }
//        }
//        return true;
//    }
//
//    void close() {
//        if (is_connected()) {
//            this->clt.stop();
//            //Serial.println("Connection closed");
//        }
//    }
//
//private:
//    WiFiClient clt;
//    char* url;
//    int port;
//    int timeout_ms;
//};


//unsigned long t = 0;
//void ssetup(void) {
//     // will pause Zero, Leonardo, etc until serial console opens
//
//    
//    
//
//    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
//    Serial.print("Accelerometer range set to: ");
//    switch (mpu.getAccelerometerRange()) {
//    case MPU6050_RANGE_2_G:
//        Serial.println("+-2G");
//        break;
//    case MPU6050_RANGE_4_G:
//        Serial.println("+-4G");
//        break;
//    case MPU6050_RANGE_8_G:
//        Serial.println("+-8G");
//        break;
//    case MPU6050_RANGE_16_G:
//        Serial.println("+-16G");
//        break;
//    }
//    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//    Serial.print("Gyro range set to: ");
//    switch (mpu.getGyroRange()) {
//    case MPU6050_RANGE_250_DEG:
//        Serial.println("+- 250 deg/s");
//        break;
//    case MPU6050_RANGE_500_DEG:
//        Serial.println("+- 500 deg/s");
//        break;
//    case MPU6050_RANGE_1000_DEG:
//        Serial.println("+- 1000 deg/s");
//        break;
//    case MPU6050_RANGE_2000_DEG:
//        Serial.println("+- 2000 deg/s");
//        break;
//    }
//
//    mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
//    Serial.print("Filter bandwidth set to: ");
//    switch (mpu.getFilterBandwidth()) {
//    case MPU6050_BAND_260_HZ:
//        Serial.println("260 Hz");
//        break;
//    case MPU6050_BAND_184_HZ:
//        Serial.println("184 Hz");
//        break;
//    case MPU6050_BAND_94_HZ:
//        Serial.println("94 Hz");
//        break;
//    case MPU6050_BAND_44_HZ:
//        Serial.println("44 Hz");
//        break;
//    case MPU6050_BAND_21_HZ:
//        Serial.println("21 Hz");
//        break;
//    case MPU6050_BAND_10_HZ:
//        Serial.println("10 Hz");
//        break;
//    case MPU6050_BAND_5_HZ:
//        Serial.println("5 Hz");
//        break;
//    }
//
//    Serial.println("");
//    delay(100);
//    t = millis();
//}
//
//void readResponse(WiFiClient* client) {
//    unsigned long timeout = millis();
//    while (client->available() == 0) {
//        if (millis() - timeout > 5000) {
//            Serial.println(">>> Client Timeout !");
//            client->stop();
//            return;
//        }
//    }
//
//    // Read all the lines of the reply from server and print them to Serial
//    while (client->available()) {
//        String line = client->readStringUntil('\r');
//        Serial.print(line);
//    }
//
//    Serial.printf("\nClosing connection\n\n");
//}
//
//void lloop() {
//
//    /* Get new sensor events with the readings */
//    sensors_event_t a, g, temp;
//    mpu.getEvent(&a, &g, &temp);
//
//    /* Print out the values */
//    Serial.print("Acceleration X: ");
//    Serial.print(a.acceleration.x);
//    Serial.print(", Y: ");
//    Serial.print(a.acceleration.y);
//    Serial.print(", Z: ");
//    Serial.print(a.acceleration.z);
//    Serial.println(" m/s^2");
//
//    Serial.print("Rotation X: ");
//    Serial.print(g.gyro.x);
//    Serial.print(", Y: ");
//    Serial.print(g.gyro.y);
//    Serial.print(", Z: ");
//    Serial.print(g.gyro.z);
//    Serial.println(" rad/s");
//
//    Serial.print("Temperature: ");
//    Serial.print(temp.temperature);
//    // Serial.print(" More: ");
//    // Serial.print(mpu.);
//    Serial.print(" More: ");
//    Serial.print(t - millis());
//    t = millis();
//    Serial.println(" degC");
//
//    Serial.println("");
//    delay(500);
//}


/*



void loop(){
  WiFiClient client;
  String footer = String(" HTTP/1.1\r\n") + "Host: " + String(host) + "\r\n" + "Connection: close\r\n\r\n";

  // WRITE --------------------------------------------------------------------------------------------
  if (!client.connect(host, httpPort)) {
	return;
  }

  client.print("GET /update?api_key=" + writeApiKey + "&field1=" + field1 + footer);
  readResponse(&client);

  // READ --------------------------------------------------------------------------------------------

  String readRequest = "GET /channels/" + channelID + "/fields/" + fieldNumber + ".json?results=" + numberOfResults + " HTTP/1.1\r\n" +
					   "Host: " + host + "\r\n" +
					   "Connection: close\r\n\r\n";

  if (!client.connect(host, httpPort)) {
	return;
  }

  client.print(readRequest);
  readResponse(&client);

  // -------------------------------------------------------------------------------------------------

  ++field1;
  delay(10000);
}
*/