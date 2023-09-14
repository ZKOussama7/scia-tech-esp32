typedef struct{
    double Ax, Ay, Az;
    double Gx, Gy, Gz;
    double T;
} Val_MPU;

static bool serial_began = false;
static bool wifi_began = false;

bool Serial_Init(int speed) {
    if (!serial_began) {
        Serial.begin(speed);
        serial_began = true;
        delay(20);
    }
    return Serial;
}

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
    S_Mpu(){}

    ~S_Mpu(){}

    bool begin() {
        bool _began = this->_mpu.begin();
        if (!_began)
            return _began;
        this->_mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
        this->_mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
        this->_mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
        
        return _began;
    }

    Val_MPU read() {
        sensors_event_t a, g, temp;
        _mpu.getEvent(&a, &g, &temp);
        Val_MPU reading;
        reading.Ax = a.acceleration.x;
        reading.Ay = a.acceleration.y;
        reading.Az = a.acceleration.z;
        reading.Gx = g.gyro.x;
        reading.Gy = g.gyro.y;
        reading.Gz = g.gyro.z;
        reading.T = temp.temperature;
        return reading;
    }

    Val_MPU read_A() {
        sensors_event_t a;
        _mpu.getAccelerometerSensor()->getEvent(&a);
        Val_MPU reading;
        reading.Ax = a.acceleration.x;
        reading.Ay = a.acceleration.y;
        reading.Az = a.acceleration.z;
        reading.Gx = 0;
        reading.Gy = 0;
        reading.Gz = 0;
        reading.T = 0;
        return reading;
    }

    Val_MPU read_G() {
        sensors_event_t g;
        _mpu.getGyroSensor()->getEvent(&g);
        Val_MPU reading;
        reading.Ax = 0;
        reading.Ay = 0;
        reading.Az = 0;
        reading.Gx = g.gyro.x;
        reading.Gy = g.gyro.y;
        reading.Gz = g.gyro.z;
        reading.T = 0;
        return reading;
    }

    Val_MPU read_T() {
        sensors_event_t temp;
        _mpu.getTemperatureSensor()->getEvent(&temp);
        Val_MPU reading;
        reading.Ax = 0;
        reading.Ay = 0;
        reading.Az = 0;
        reading.Gx = 0;
        reading.Gy = 0;
        reading.Gz = 0;
        reading.T = temp.temperature;
        return reading;
    }

private:
    MPU6050 _mpu;
};

class C_Raw
{
public:
    C_Raw(char* url_input, int port_input, int timeout_ms=2000){
        this->url = url_input;
        this->port = port_input;
        this->timeout_ms = timeout_ms;
    }
    C_Raw(const char* url_input, int port_input, int timeout_ms = 2000) :
        url((char*)url_input), port(port_input), timeout_ms(timeout_ms) {}


    ~C_Raw(){}

    bool begin() {
        if (!is_connected()) 
            this->clt.connect(this->url, this->port, this->timeout_ms);
        return is_connected();
    }

    bool is_connected() {
        return this->clt.connected();
    }

    bool is_available() {
        return this->clt.available();
    }

    bool writeLine(String Text){
        if (!is_connected())
            return false;
        this->clt.println(Text);
        return true;
    }

    String readLine(){
        if (!is_connected())
            return "ERROR: !connected";
        
        if (!is_available())
            return "ERROR: !available";

        String Line;
        Line = clt.readStringUntil('\r');
        return Line;

        // needs to be corrected
        //while (clt.available() == 0) {
        //    if (millis() - timeout > 5000) {
        //        //Serial.println(">>> Client Timeout !");
        //        clt.stop();
        //        return;
        //    }
        //}

        // Read all the lines of the reply from server and print them to Serial
       /* while (clt.available()) {
            String line = clt.readStringUntil('\r');
            Serial.print(line);
        }*/

        //Serial.printf("\nClosing connection\n\n");
    }

    bool waitAvailable() {
        int timeout = millis();
        while (!is_available()) {
            if (millis() - timeout > this->timeout_ms) {
                //Serial.println(">>> Client Timeout !");
                //clt.stop();
                return false;
            }
        }
        return true;
    }

    void close() {
        if (is_connected()) {
            this->clt.stop();
            //Serial.println("Connection closed");
        }
    }

private:
    WiFiClient clt;
    char* url;
    int port;
    int timeout_ms;
};


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