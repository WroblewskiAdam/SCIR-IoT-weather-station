#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include <PubSubClient.h>
#include <mqtt_secrets.h>

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

AsyncWebServer web_server(80);

// #define wifi_ssid "Orange_Gosc_7AD0"
// #define wifi_password "dupadupa"
// #define wifi_ssid "Orange_Swiatlowod_7AD0"
// #define wifi_password "7adkKdbCs7PHKp4ea3"

#define wifi_ssid "zetor"
#define wifi_password "ursus360"
#define wifi_timeout 60000


// PubSubClient MQTT_CLIENT;
PubSubClient MQTT_client(wifi_client);
const char* mqtt_server = "mqtt3.thingspeak.com";
const char* publishTopic ="channels/2463135/publish"; 


#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
float bme_temperature;
float bme_humidity;
float bme_pressure;
float bme_altitude;

Adafruit_MCP9808  mcp;
float mcp_temperature;

const int yl_digital_pin = 16;
const int yl_analog_pin = 32;
float rain_analog;
bool rain = false;

unsigned long last_upload_time = 0;
unsigned long upload_interval = 15000;
String ip_address = "192.168.10.201";
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
    Serial.println(rain);
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

    WebSerial.print(bme_pressure);
    WebSerial.print(" hPa");
    WebSerial.print(" | ");
    delay(100);

    WebSerial.print(bme_humidity);
    WebSerial.print(" %");
    WebSerial.print(" | ");
    delay(100);

    WebSerial.print(rain);
    WebSerial.print(" rain");
    delay(100);
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

void getBMEvalues() {
    bme_temperature = bme.readTemperature();
    bme_pressure = bme.readPressure() / 100.0F;
    bme_altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bme_humidity = bme.readHumidity();
}

void getMCPvalues()
{
    mcp_temperature = mcp.readTempC() - offset;
}

void getYLvalues(){
    rain_analog = analogRead(yl_analog_pin);
    Serial.println(rain_analog);
    rain_analog < 850 ? rain = true : rain = false; 
}

void MQTTreconnect() {
    // Loop until we're reconnected
    while (!MQTT_client.connected()) {
        Serial.println("Connecting with MQTT broker...");
        if (MQTT_client.connect(SECRET_MQTT_USERNAME, SECRET_MQTT_CLIENT_ID, SECRET_MQTT_PASSWORD)) 
        {  
            Serial.println("connected");
        } 
        else 
        {
            Serial.println("failed connecting to MQTT broker, ");
            Serial.print(MQTT_client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void send_data_to_cloud_https()
{
    ThingSpeak.setField(1, mcp_temperature);
    ThingSpeak.setField(2, bme_pressure);
    ThingSpeak.setField(3, bme_humidity);
    // ThingSpeak.setField(4, rainfall_value);
    // ThingSpeak.setField(5, rain_indicator);
    
    ThingSpeak.writeFields(channel_id,write_api_key);
    Serial.println("SENT DATA TO CLOUD");
}

void send_data_to_cloud_MQTT()
{
    String dataText = String("field1=" + String(mcp_temperature) +
                            "&field2=" + String(bme_pressure) +
                            "&field3=" + String(bme_humidity) +
                            "&field4=" + String(rain) +
                            "&status=MQTTPUBLISH");

    
    if (MQTT_client.publish(publishTopic, dataText.c_str()))
        Serial.println("Message published ["+String(publishTopic)+"]: " + dataText);
    else Serial.println("Error publishing");
}

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    MQTT_client.setServer(mqtt_server, 1883);
    initWiFi();
   
    WebSerial.begin(&web_server);
    WebSerial.msgCallback(recvMsg);
    web_server.begin();

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
    
    if(WiFi.localIP().toString() == "0.0.0.0") {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Connection lost...");
        ESP.restart();
    }
    else{
        digitalWrite(LED_BUILTIN, HIGH);
    
        if(!MQTT_client.connected()) MQTTreconnect();
        else
        {
            MQTT_client.loop(); 

            if(millis() - last_upload_time > upload_interval)
            {
                getBMEvalues();
                getMCPvalues();
                getYLvalues();

                print_values();
                print_web_values();
                
                send_data_to_cloud_MQTT();
            
                last_upload_time = millis();
            }
        }
    }    
}

