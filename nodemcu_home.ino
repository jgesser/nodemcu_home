#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <BlynkSimpleEsp8266.h>
#include <Adafruit_MQTT_Client.h>
#include <FS.h>
#include <OTA.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "parameters.h" // file not added to git cause it has credentials in it.

#define PIN_RESET 0
#define PIN_DHT D2
#define PIN_BME_SCL D3
#define PIN_BME_SDA D4
#define PIN_DS18B20 D7
#define PIN_RELAY1 D5
#define PIN_RELAY2 D6
#define PIN_LED D1

#define BLYNK_PIN_INTERVAL V0
#define BLYNK_PIN_DS18B20 V1
#define BLYNK_PIN_DHT_TEMP V2
#define BLYNK_PIN_DHT_HUM V3
#define BLYNK_PIN_BME_TEMP V4
#define BLYNK_PIN_BME_PRES V5
#define BLYNK_PIN_BME_HUM V6
#define BLYNK_PIN_BME_ALT V7
#define BLYNK_PIN_RELAY1 V8
#define BLYNK_PIN_RELAY2 V9
#define BLYNK_PIN_RESET V10

OTA ota;

BlynkTimer timer;

WiFiClient client;

Adafruit_MQTT_Client mqtt(&client, MQTT_HOST, MQTT_PORT);
Adafruit_MQTT_Subscribe mqttSubRelay1 = Adafruit_MQTT_Subscribe(&mqtt, "nodemcu1/relay1");
Adafruit_MQTT_Publish mqttPubRelay1 = Adafruit_MQTT_Publish(&mqtt, "nodemcu1/relay1");
//Adafruit_MQTT_Subscribe mqttSubRelay2 = Adafruit_MQTT_Subscribe(&mqtt, "nodemcu1/relay2");
//Adafruit_MQTT_Publish  mqttPubRelay3 = Adafruit_MQTT_Publish(&mqtt, "nodemcu1/relay2");
Adafruit_MQTT_Publish mqttPubTemp = Adafruit_MQTT_Publish(&mqtt, "nodemcu1/temp");
Adafruit_MQTT_Publish mqttPubHumidity = Adafruit_MQTT_Publish(&mqtt, "nodemcu1/humidity");

//Adafruit_MQTT_Client mqtt(&client, MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS);
//Adafruit_MQTT_Subscribe mqttSubRelay1 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USER "/f/nodemcu1.relay1");
//Adafruit_MQTT_Publish mqttPubRelay1 = Adafruit_MQTT_Publish(&mqtt, MQTT_USER "/f/nodemcu1.relay1");
////Adafruit_MQTT_Subscribe mqttSubRelay2 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USER "/f/nodemcu1.relay2");
////Adafruit_MQTT_Publish  mqttPubRelay3 = Adafruit_MQTT_Publish(&mqtt, MQTT_USER "/f/nodemcu1.relay2");
//Adafruit_MQTT_Publish mqttPubTemp = Adafruit_MQTT_Publish(&mqtt, MQTT_USER "/f/nodemcu1.temp");
//Adafruit_MQTT_Publish mqttPubHumidity = Adafruit_MQTT_Publish(&mqtt, MQTT_USER "/f/nodemcu1.humidity");

Adafruit_BMP280 bme;

DHT dht(PIN_DHT, DHT22);

OneWire oneWire(PIN_DS18B20);
DallasTemperature ds18b20(&oneWire);

bool relay1State;
int tempTimer = -1;

void setup() {
    Serial.begin(115200);
    Serial.println("\nSETUP BEGIN");

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW); // led off

    setupWifi();
    setupSensors();
    setupRelays();
    setupMqtt();
    setupBlynk();
    ota.setup();

    Serial.println("\nSETUP END");
}

void setupSensors() {
    Wire.begin(PIN_BME_SDA, PIN_BME_SCL);
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    }
    dht.begin();
    ds18b20.begin();
}

void setupRelays() {
    Serial.println("Initializing Relays...");

    pinMode(PIN_RELAY1, OUTPUT);
    pinMode(PIN_RELAY2, OUTPUT);
    SPIFFS.begin();
    restoreRelayState();

    Serial.println("DONE!");
}

void mqttRelay1Callback(char *str, uint16_t len) {
    Serial.print("onoff: ");
    Serial.println((char*) (mqttSubRelay1.lastread));
    //            state = strcasecmp((char*) onoff.lastread, "ON");
    toggleRelayState();
}

void setupMqtt() {
    Serial.println("Initializing MQTT...");

    mqttSubRelay1.setCallback(mqttRelay1Callback);
    mqtt.subscribe(&mqttSubRelay1);

    Serial.println("DONE!");
}

void setupWifi() {
    Serial.println("Initializing WifiManager...");

    WiFiManager wifiManager;
    wifiManager.setConfigPortalTimeout(300); //5 min.
    wifiManager.autoConnect();
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    Serial.println("DONE!");
}

void setupBlynk() {
    Serial.println("Initializing Blynk...");

//    Blynk.begin(BLINK_TOKEN, WiFi.SSID().c_str(), WiFi.psk().c_str());
    Blynk.begin(BLINK_TOKEN, "", "");

    Serial.println("DONE!");
}

void loop() {
    ota.loop();
    mqttLoop();
    timer.run();
    Blynk.run();
}

void mqttLoop() {

    mqttConnect();
    mqtt.processPackets(1000);

    //Read from our subscription queue until we run out, or
    //wait up to 5 seconds for subscription to update
//    Adafruit_MQTT_Subscribe* subscription;
//    while ((subscription = mqtt.readSubscription(1000))) {
//        //If we're in here, a subscription updated...
//        if (subscription == &mqttSubRelay1) {
//            //Print the new value to the serial monitor
//            Serial.print("onoff: ");
//            Serial.println((char*) (mqttSubRelay1.lastread));
//            //            state = strcasecmp((char*) onoff.lastread, "ON");
//            toggleRelayState();
//        }
//    }
}

void mqttConnect() {
    int8_t ret;
    // Stop if already connected
    if (mqtt.connected()) {
        return;
    }

    Serial.print("Connecting to MQTT... ");
    uint8_t retries = 10;
    while ((ret = mqtt.connect()) != 0) // connect will return 0 for connected
    {
        Serial.println(mqtt.connectErrorString(ret));
        Serial.println("Retrying MQTT connection in 5 seconds...");
        mqtt.disconnect();
        delay(5000);  // wait 5 seconds
        retries--;
        if (retries == 0) {
            // basically die and wait for WDT to reset me
            while (1)
                ;
        }
    }
    Serial.println("MQTT Connected!");
}

void toggleRelayState() {
    relay1State = !relay1State;
    applyRalayState();
    saveRelayState();
}

void applyRalayState() {
    Serial.print("Changing relay state to: ");
    Serial.println(relay1State);
    if (relay1State) {
        digitalWrite(PIN_RELAY1, LOW);
    } else {
        digitalWrite(PIN_RELAY1, HIGH);
    }
}

void saveRelayState() {
    File file = SPIFFS.open("/state.txt", "w+");
    if (!file) {
        Serial.println("Error saving state");
    } else {
        file.write(relay1State);
        Serial.print("State saved: ");
        Serial.println(relay1State);
    }
    file.close();
}

void restoreRelayState() {
    File file = SPIFFS.open("/state.txt", "r");
    if (!file) {
        Serial.println("Error reading state");
    }
    relay1State = file.read();
    Serial.print("State read: ");
    Serial.println(relay1State);
    file.close();

    applyRalayState();
}

void onTimer() {

    digitalWrite(PIN_LED, HIGH); // led off

    readDHT22();
    readBME280();
    readDS18B20();

    digitalWrite(PIN_LED, LOW); // led on
}

void readBME280() {
    Serial.println();
    Serial.println("-- BMP280 --");

    float temp = bme.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");
    if (!isnan(temp) && temp > -100) {
        Blynk.virtualWrite(BLYNK_PIN_BME_TEMP, temp);
    }

    //    float fTemp = bme.readTemperature() * 9.0 / 5.0 + 32;
    //    Serial.print(fTemp);
    //    Serial.println(" *F");

    float pres = bme.readPressure() / 100.0F;
    Serial.print("Pressure = ");
    Serial.print(pres);
    Serial.println(" hPa");
    if (!isnan(pres) && pres > -100) {
        Blynk.virtualWrite(BLYNK_PIN_BME_PRES, pres);
    }

    float alt = bme.readAltitude();
    Serial.print("Approx. Altitude = ");
    Serial.print(alt);
    Serial.println(" m");
    if (!isnan(alt) && alt > -100) {
        Blynk.virtualWrite(BLYNK_PIN_BME_ALT, alt);
    }

    //    Serial.print("Humidity = ");
    //    Serial.print(bme.readHumidity());
    ////    float rH = bme.readHumidity();
    ////    Serial.print(rH);
    //    Serial.println(" %");

    //    Serial.print("Dew Point = ");
    //    float dPoint = fTemp - ((100 - rH) / 5);
    //    Serial.print(dPoint);
    //    Serial.println(" *F");
    //
    //    Serial.print("Heat Index = ");
    //    float hIndex = 0.5 * (fTemp + 61.0 + ((fTemp - 68.0) * 1.2) + (rH * 0.094));
    //    Serial.print(hIndex);
    //    Serial.println(" *F");
}

void readDHT22() {
    Serial.println();
    Serial.println("-- DHT22 --");

    float temp = dht.readTemperature();
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");
    if (!isnan(temp) && temp > -100) {
        Blynk.virtualWrite(BLYNK_PIN_DHT_TEMP, temp);

        mqttPubTemp.publish(temp);
    }

    float hum = dht.readHumidity();
    Serial.print("Humidity = ");
    Serial.print(hum);
    Serial.println(" %");
    if (!isnan(hum) && hum > -100) {
        Blynk.virtualWrite(BLYNK_PIN_DHT_HUM, hum);

        mqttPubHumidity.publish(hum);
    }
}

void readDS18B20() {
    Serial.println();
    Serial.println("-- DS18B20 --");

    float temp = ds18b20.getTempCByIndex(0);

    ds18b20.requestTemperatures();
    Serial.print("Temperature = ");
    Serial.print(temp);
    Serial.println(" *C");
    if (!isnan(temp) && temp > -100) {
        Blynk.virtualWrite(BLYNK_PIN_DS18B20, temp);
    }
}

BLYNK_CONNECTED() {
    Blynk.syncVirtual(BLYNK_PIN_INTERVAL);
}

BLYNK_WRITE(BLYNK_PIN_RELAY1) {
    Serial.print("BLYNK: Relay 1 changed");

    //mqttPubRelay1.publish(relay1State ? "OFF" : "ON");

    toggleRelayState();
}

BLYNK_WRITE(BLYNK_PIN_INTERVAL) {
    Serial.print("BLYNK: Refresh interval changed: ");
    Serial.println(param.asInt());

    int interval = param.asInt() * 1000;
    if (tempTimer > -1) {
        timer.deleteTimer(0);
    }
    tempTimer = timer.setInterval(interval, onTimer);
}

BLYNK_WRITE(BLYNK_PIN_RESET) {
    Serial.print("BLYNK: BLYNK_PIN_RESET: ");
    Serial.println(param.asInt());

    if (param.asInt() != 2) {
        Serial.println("Restarting... Bye ;-)");
        ESP.restart();
    } else {
        Serial.println("Reseting... :-O");
        ESP.reset();
    }
}

