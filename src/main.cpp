#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_MCP9808.h>

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#define channel_id 2463135
#define write_api_key "V5HXBBRF8G4HKIDI"
WiFiClient wifi_client;

AsyncWebServer server(80);

// #define wifi_ssid "Orange_Gosc_7AD0"
// #define wifi_password "dupadupa"
#define wifi_ssid "Orange_Swiatlowod_7AD0"
#define wifi_password "7adkKdbCs7PHKp4ea3"

// #define wifi_ssid "zetor"
// #define wifi_password "ursus360"
#define wifi_timeout 60000

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
float bme_temperature;
float bme_humidity;
float bme_pressure;
float bme_altitude;

Adafruit_MCP9808  mcp;
float mcp_temperature;

const int yl_digital_pin = 16;
const int yl_analog_pin = 4 ;

unsigned long rain_last_impulse_millis = 0;
unsigned long rain_interval = 900000;
float mm_per_tick = 0.217;
float rainfall_value = 0;
bool rain = false;
float rain_indicator = 0;
int impulse_counter = 0;
bool impulse = false;
bool prev_impulse_val = false;


unsigned long upload_previousMillis = 0;
unsigned long upload_interval = 1500;
String ip_address = "192.168.1.30";
float offset = 0.0;

void initWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifi_ssid, wifi_password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < wifi_timeout) 
    {
        Serial.println("Connecting to WiFi ...");
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
    }
    
    if(WiFi.status() == WL_CONNECTED){
        Serial.println("Connected to WiFi");
        Serial.println(WiFi.localIP());
    }
    else{
        Serial.println("CONNECTING TO WIFI FAILED - restarting ESP32");
        digitalWrite(LED_BUILTIN, LOW);
        delay(5000);
        ESP.restart();
    }
}


void print_values() {
    Serial.println();
    Serial.print("MCP TEMP = ");
    Serial.print(mcp_temperature);
    Serial.println(" *C");

    Serial.print("Temperature = ");
    Serial.print(bme_temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(bme_pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme_altitude);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(bme_humidity);
    Serial.println(" %");
    
    Serial.print("RainFall = ");
    Serial.print(rainfall_value);
    Serial.println(" mm/m^2");
}

void print_web_values() {
    WebSerial.println();
    WebSerial.print("MCP temp = ");
    WebSerial.print(mcp_temperature);
    WebSerial.print(" *C");
    WebSerial.print(" | ");
    delay(100);
    

    WebSerial.print("MBE temp = ");
    WebSerial.print(bme_temperature);
    WebSerial.print(" *C");
    WebSerial.print(" | ");
    delay(100);

    // WebSerial.print("Pre = ");
    WebSerial.print(bme_pressure);
    WebSerial.print(" hPa");
    WebSerial.print(" | ");
    delay(100);

    // // WebSerial.print("Approx. Altitude = ");
    // WebSerial.print(bme_altitude);
    // WebSerial.print(" m");
    // WebSerial.print(" | ");
    // delay(100);

    // WebSerial.print("Humidity = ");
    WebSerial.print(bme_humidity);
    WebSerial.print(" %");
    delay(100);
    
    // WebSerial.print("RainFall = ");
    // WebSerial.print(rainfall_value);
    // WebSerial.println(" mm/m^2");
}


void getBMEvalues() {
    bme_temperature = bme.readTemperature();
    bme_pressure = bme.readPressure() / 100.0F;
    bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bme_humidity = bme.readHumidity();
}


void getMCPvalues()
{
    // mcp.wake();
    mcp_temperature = mcp.readTempC() - offset;
    // mcp.shutdown_wake(1);
}


void rainMeter()
{
    prev_impulse_val = impulse;
    impulse = (analogRead(32) < 2350) ? true : false;
    while(impulse)
    {
        prev_impulse_val = impulse;
        impulse = (analogRead(32) < 2350) ? true : false;
        // Serial.println("waiting");
    }
    if(prev_impulse_val == 1 && impulse == 0) 
    {
        Serial.println("impulse");
        Serial.println();
        delay(20);
        impulse_counter = impulse_counter + 1;
        rainfall_value = impulse_counter * mm_per_tick;
        
        rain = true;
        rain_indicator = 1;
        rain_last_impulse_millis = millis();
    }
    if(rain = true && millis() - rain_last_impulse_millis > rain_interval)
    {
        rain = false;
        rain_indicator = 0;
        rainfall_value = 0;
    }
}


void recvMsg(uint8_t *data, size_t len){
    WebSerial.println("Received Data...");
    String d = "";
    for(int i=0; i < len; i++){
        d += char(data[i]);
    }
    WebSerial.println(d);
    if (d == "reset"){
        Serial.println("web reset");
        WebSerial.println("web reset");
        delay(2000);
        ESP.restart();
    }
    if (d.toFloat()){
        offset = d.toFloat();
        WebSerial.println("New offset value ");
        WebSerial.print(offset);
        Serial.println("New offset value ");
        Serial.print(offset);
    }
}

void send_data_to_cloud()
{
    ThingSpeak.setField(1, mcp_temperature);
    ThingSpeak.setField(2, bme_pressure);
    ThingSpeak.setField(3, bme_humidity);
    // ThingSpeak.setField(4, rainfall_value);
    // ThingSpeak.setField(5, rain_indicator);
    
    ThingSpeak.writeFields(channel_id,write_api_key);
    Serial.println("SENT DATA TO CLOUD");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // initWiFi();

    // ThingSpeak.begin(wifi_client);

    // WebSerial.begin(&server);
    // WebSerial.msgCallback(recvMsg);
    // server.begin();
    
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
    else{
        Serial.println("BME280 detected");
        Serial.println();
    }   

    if (!mcp.begin(0x18)) {
        Serial.println("Could not find a valid MCP9808 sensor, check wiring!");
    }
    else{
        Serial.println("MCP9808 detected");
        Serial.println();
        mcp.printSensorDetails();
    }
}


void loop() {
    digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
    
    // if(WiFi.localIP().toString() == "0.0.0.0") {
    //     digitalWrite(LED_BUILTIN, LOW);
    //     Serial.println("Connection lost...");
    //     ESP.restart();
    // }
    
    // if(WiFi.localIP().toString() == ip_address) {
        digitalWrite(LED_BUILTIN, HIGH);
        
        unsigned long upload_current_millis = millis();

        // rainMeter();
        // delay(10);

        if(millis() - upload_previousMillis > upload_interval)
        {
            getBMEvalues();
            getMCPvalues();
            print_values();
            // print_web_values();
            // send_data_to_cloud();
            Serial.print("rain digital: ");
            Serial.println(digitalRead(yl_digital_pin));
            Serial.print("rain analog: ");
            Serial.println(analogRead(yl_analog_pin));
            Serial.println();
            Serial.println();
            Serial.println();
            upload_previousMillis = millis();
        }
    // }
}

