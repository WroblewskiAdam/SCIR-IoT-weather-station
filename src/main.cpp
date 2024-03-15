#include <Arduino.h>
#include <WiFi.h>
#include <ThingSpeak.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


#define channel_id 2463135
#define write_api_key "V5HXBBRF8G4HKIDI"
WiFiClient wifi_client;

#define wifi_ssid "Orange_Gosc_7AD0"
#define wifi_password "dupadupa"
#define wifi_timeout 60000

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C
float temperature;
float humidity;
float pressure;
float altitude;


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
unsigned long upload_interval = 60000;

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


void printBMEvalues() {
    Serial.println();
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" *C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" hPa");

    Serial.print("Approx. Altitude = ");
    Serial.print(altitude);
    Serial.println(" m");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");
    
    Serial.print("RainFall = ");
    Serial.print(rainfall_value);
    Serial.println(" mm/m^2");

}


void getBMEvalues() {
    temperature = bme.readTemperature();
    pressure = bme.readPressure() / 100.0F;
    altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
    humidity = bme.readHumidity();
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

void setup() {
    Serial.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);
    
    initWiFi();

    ThingSpeak.begin(wifi_client);
    
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
    else{
        Serial.println("BME280 detected");
        Serial.println();
    }   
}


void loop() {
    // Serial.println(WiFi.localIP());
    // Serial.print("wifi status: ");
    // Serial.println(WiFi.status());
    // Serial.println();
    digitalWrite(LED_BUILTIN, WiFi.status() == WL_CONNECTED);
    
    if(WiFi.localIP().toString() == "0.0.0.0") {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("Connection lost...");
        ESP.restart();
    }
    
    if(WiFi.localIP().toString() == "192.168.144.3") {
        digitalWrite(LED_BUILTIN, HIGH);
        
        unsigned long upload_current_millis = millis();

        rainMeter();
        // delay(10);

        if(millis() - upload_previousMillis > 15000)
        {
            getBMEvalues();
            printBMEvalues();
            ThingSpeak.setField(1, temperature);
            ThingSpeak.setField(2, pressure);
            ThingSpeak.setField(3, humidity);
            ThingSpeak.setField(4, rainfall_value);
            
            ThingSpeak.setField(5, rain_indicator);
            
            ThingSpeak.writeFields(channel_id,write_api_key);
            Serial.println("SENT DATA TO CLOUD");

            upload_previousMillis = millis();
        }
    }
}

