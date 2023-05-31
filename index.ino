#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include <Wire.h>
#include "SPIFFS.h"
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "twilio.hpp"

char* text;
char* number;
bool error;

// Replace with your network credentials
const char *ssid = "WIFI_ID";
const char *password = "PASSWORD";

//IFTTT
void send_event(const char *event);
const char *host = "maker.ifttt.com";
const char *privateKey = "PRIVATE_KEY";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

Twilio *twilio;
static const char *account_sid = "Auth-ID";
static const char *auth_token = "Auth-Token";
static const char *from_number = "FOM_NUMBER";
static const char *to_number = "TO_NUMBER";
static const char *message = "Fall Detected";
 
// Json Variable to Hold Sensor Readings
JSONVar readings;

//Blynk
#define BLYNK_TEMPLATE_ID "TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "IoT"
#define BLYNK_AUTH_TOKEN "AUTH_TOKEN"
#define BLYNK_PRINT Serial

// Timer variables
unsigned long lastTime = 0;
unsigned long lastTimeTemperature = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 10;
unsigned long temperatureDelay = 1000;
unsigned long accelerometerDelay = 200;

const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;

boolean fall = false;     // stores if a fall has occurred
boolean trigger1 = false; // stores if first trigger (lower threshold) has occurred
boolean trigger2 = false; // stores if second trigger (upper threshold) has occurred
boolean trigger3 = false; // stores if third trigger (orientation change) has occurred
byte trigger1count = 0;   // stores the counts past since trigger 1 was set true
byte trigger2count = 0;   // stores the counts past since trigger 2 was set true
byte trigger3count = 0;   // stores the counts past since trigger 3 was set true
int angleChange = 0;
int flag = 0;
int count = 0;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

// Gyroscope sensor deviation
float gyroXerror = 0.15;
float gyroYerror = -0.42;
float gyroZerror = 0.11;

int blynk_final = 0;

String get_wifi_status(int status)
{
    switch (status)
    {
    case WL_IDLE_STATUS:
        return "WL_IDLE_STATUS";
    case WL_SCAN_COMPLETED:
        return "WL_SCAN_COMPLETED";
    case WL_NO_SSID_AVAIL:
        return "WL_NO_SSID_AVAIL";
    case WL_CONNECT_FAILED:
        return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST:
        return "WL_CONNECTION_LOST";
    case WL_CONNECTED:
        return "WL_CONNECTED";
    case WL_DISCONNECTED:
        return "WL_DISCONNECTED";
    }
}

// Init MPU6050
void initMPU()
{
    if (!mpu.begin())
    {
        Serial.println("Failed to find MPU6050 chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");
}

void initSPIFFS()
{
    if (!SPIFFS.begin())
    {
        Serial.println("An error has occurred while mounting SPIFFS");
    }
    Serial.println("SPIFFS mounted successfully");
}

void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");

    int n = WiFi.scanNetworks();
    Serial.println("Scanning Near Wifi Networks");
    if (n == 0)
    {
        Serial.println("0 Networks Found");
    }

    else
    {
        Serial.print(n);
        Serial.println(" Networks Found");
        for (int i = 0; i < n; ++i)
        {
            // Print SSID and RSSI for each network found
            Serial.print(i + 1);
            Serial.print(": ");
            Serial.print(WiFi.SSID(i));
            Serial.print(" (");
            Serial.print(WiFi.RSSI(i));
            Serial.print(")");
            Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
            delay(10);
        }
    }
    Serial.println("");

    Serial.print("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(1000);
    }
    Serial.println("");
    Serial.println(WiFi.localIP());
}

String getGyroReadings()
{
    mpu.getEvent(&a, &g, &temp);

    float gyroX_temp = g.gyro.x;
    if (abs(gyroX_temp) > gyroXerror)
    {
        gyroX += gyroX_temp / 50.00;
    }

    float gyroY_temp = g.gyro.y;
    if (abs(gyroY_temp) > gyroYerror)
    {
        gyroY += gyroY_temp / 70.00;
    }

    float gyroZ_temp = g.gyro.z;
    if (abs(gyroZ_temp) > gyroZerror)
    {
        gyroZ += gyroZ_temp / 90.00;
    }

    readings["gyroX"] = String(gyroX);
    readings["gyroY"] = String(gyroY);
    readings["gyroZ"] = String(gyroZ);

    String jsonString = JSON.stringify(readings);
    return jsonString;
}

String getAccReadings()
{
    mpu.getEvent(&a, &g, &temp);
    // Get current acceleration values
    accX = a.acceleration.x + 11.4;
    accY = a.acceleration.y + 19.61;
    accZ = a.acceleration.z + 19.61;

    AcY = accY;

    readings["accX"] = String(accX);
    readings["accY"] = String(accY);
    readings["accZ"] = String(accZ);
    String accString = JSON.stringify(readings);
    return accString;
}

String getTemperature()
{
    mpu.getEvent(&a, &g, &temp);
    temperature = temp.temperature;
    return String(temperature);
}

void setup()
{
    Serial.begin(115200);
    Serial2.begin(115200);
    initWiFi();
    initSPIFFS();
    initMPU();

    // Handle Web Server
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html", "text/html"); });

    server.serveStatic("/", SPIFFS, "/");

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK"); });

    server.on("/resetX", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    gyroX=0;
    request->send(200, "text/plain", "OK"); });

    server.on("/resetY", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    gyroY=0;
    request->send(200, "text/plain", "OK"); });

    server.on("/resetZ", HTTP_GET, [](AsyncWebServerRequest *request)
              {
    gyroZ=0;
    request->send(200, "text/plain", "OK"); });

    // Handle Web Server Events
    events.onConnect([](AsyncEventSourceClient *client)
                     {
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }

    client->send("hello!", NULL, millis(), 10000); });
    server.addHandler(&events);

    server.begin();
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
}

void loop()
{
    Blynk.run();
    
    if ((millis() - lastTime) > gyroDelay)
    {
        // Send Events to the Web Server with the Sensor Readings
        events.send(getGyroReadings().c_str(), "gyro_readings", millis());
        lastTime = millis();
    }

    if ((millis() - lastTimeAcc) > accelerometerDelay)
    {
        // Send Events to the Web Server with the Sensor Readings
        events.send(getAccReadings().c_str(), "accelerometer_readings", millis());
        lastTimeAcc = millis();
    }

    if ((millis() - lastTimeTemperature) > temperatureDelay)
    {
        // Send Events to the Web Server with the Sensor Readings
        events.send(getTemperature().c_str(), "temperature_reading", millis());
        lastTimeTemperature = millis();
    }

      mpu_read();
         ax = (AcX - 2050) / 16384.00;
         ay = (AcY - 77) / 16384.00;
         az = (AcZ - 1947) / 16384.00;
         gx = (GyX + 270) / 131.07;
         gy = (GyY - 351) / 131.07;
         gz = (GyZ + 136) / 131.07;

        // calculating Amplitute vactor for 3 axis
        float raw_amplitude = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
        int amplitude = raw_amplitude * 10; // Mulitiplied by 10 bcz values are between 0 to 1

        if (amplitude <= 2 && trigger2 == false)
        { // if AM breaks lower threshold (0.4g)
            trigger1 = true;
            Serial.println("TRIGGER 1 ACTIVATED");
        }

        if (trigger1 == true)
        {
            trigger1count++;
            if (amplitude >= 12)
            { // if AM breaks upper threshold (3g)
                trigger2 = true;
                Serial.println("TRIGGER 2 ACTIVATED");
                trigger1 = false;
                trigger1count = 0;
            }
        }

        if (trigger2 == true)
        {
            trigger2count++;
            angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
            Serial.println(angleChange);
            if (angleChange >= 30 && angleChange <= 400)
            { // if orientation changes by between 80-100 degrees
                trigger3 = true;
                trigger2 = false;
                trigger2count = 0;
                Serial.println(angleChange);
                Serial.println("TRIGGER 3 ACTIVATED");
            }
        }
        if (trigger3 == true)
        {
            trigger3count++;
            if (trigger3count >= 10)
            {
                angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
                Serial.println(angleChange);
                if ((angleChange >= 0) && (angleChange <= 10))
                { // if orientation changes remains between 0-10 degrees
                    fall = true;
                    trigger3 = false;
                    trigger3count = 0;
                    Serial.println(angleChange);
                }
                else
                { // user regained normal orientation
                    trigger3 = false;
                    trigger3count = 0;
                    Serial.println("TRIGGER 3 DE-ACTIVATED");
                }
            }
        }

        if (fall == true)
        {
            // in event of a fall detection
            Serial.println("FALL DETECTED");
            send_event("Fall Detection");
            fall = false;
        }

        if (trigger2count >= 6)
        {
            // allow 0.5s for orientation change
            trigger2 = false;
            trigger2count = 0;
            Serial.println("TRIGGER 2 DE-ACTIVATED");
        }

        if (trigger1count >= 6)
        {
            // allow 0.5s for AM to break upper threshold
            trigger1 = false;
            trigger1count = 0;
            Serial.println("TRIGGER 1 DE-ACTIVATED");
        }

        delay(100);

     if(AcY > 0.5 && flag == 0)
     {
         Serial.println("FALL DETECTED");
         send_event("Fall Detection");
         flag = 1;
     }

     count++;
     if(count >= 60)
     {
      count = 0;
      flag = 0;
     }

     if(blynk_final == 0)
     {
      Blynk.virtualWrite(V0, "Not Detected");
     }

     else{
      Blynk.virtualWrite(V0, "Fall Detected");
     }
     
     Blynk.virtualWrite(V1, gx);
     Blynk.virtualWrite(V2, gy);
     Blynk.virtualWrite(V3, gz);
}

void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    Serial2.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (Serial2.available())
  {
    Serial.write(Serial2.read());//Forward what Software Serial received to Serial Port
  }
}


void sim800l()
{
    Serial2.println("AT+CMGF=1"); // Configuring TEXT mode
    updateSerial();
    Serial2.println("AT+CMGS=\"TO_NUMBER\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
    updateSerial();
    Serial2.print("Fall Detected! Location: LOCATION"); //text content
    updateSerial();
    Serial.println();
    Serial.println("Message Sent");
    Serial2.write(26);
}

int working = 0;

BLYNK_CONNECTED() 
{ // checks if there is a connection to Blynk.Cloud  
      Blynk.syncVirtual(V4); // get the latest value
}

BLYNK_WRITE(V4) // this command is listening when something is written to V1
{
  int pinValue = param.asInt(); // assigning incoming value from pin V1 to a variable
  
  if (pinValue = 1)
  {
    blynk_final = 1;
    send_event("Fall Detection");
    twilio = new Twilio(account_sid, auth_token);
    delay(1000);
    String response;
    bool success = twilio->send_message(to_number, from_number, message, response);
    if (success) 
    {
      Serial.println("Sent message successfully!");
    } 
    
    else 
    {
      Serial.println(response);
    }

    if(working == 1)
    {
        sim800l();
    }

    else
    {
        Serial.println("SIM800L not working");
    }
  } 
}

void mpu_read()
{
    Wire.beginTransmission(MPU_addr);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
    AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void send_event(const char *event)
{
    Serial.print("Connecting to ");
    Serial.println(host);
    WiFiClient client;
    const int httpPort = 80;

    if (!client.connect(host, httpPort))
    {
        Serial.println("Connection failed");
        return;
    }

    String url = "/trigger/";
    url += event;
    url += "/with/key/";
    url += privateKey;
    Serial.print("Requesting URL: ");
    Serial.println(url);
    client.print(String("GET ") + url + " HTTP/1.1\r\n" + "Host: " + host + "\r\n" + "Connection: close\r\n\r\n");

    while (client.connected())
    {
        if (client.available())
        {
            String line = client.readStringUntil('\r');
            Serial.print(line);
        }
        else
        {
            delay(50);
        };
    }

    Serial.println();
    Serial.println("Closing Connection");
    client.stop();
}
