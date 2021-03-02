#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <BME280I2C.h>
#include <Wire.h>
#include <coredecls.h>         // crc32()
#include <PolledTimeout.h>
#include <include/WiFiState.h> // WiFiState structure details


//**************************************************************************************************
//                              WiFi Configuration
//**************************************************************************************************
const char* AP_SSID = "your_ssid";  // your router's SSID here
const char* AP_PASS = "your_password";  // your router's password here
IPAddress staticIP(192, 168, 1, 120); // parameters below are for your static IP address, if used
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns1(0, 0, 0, 0);
IPAddress dns2(0, 0, 0, 0);
uint32_t timeout = 30E3;  // 30 second timeout on the WiFi connection



//**************************************************************************************************
//                          MQTT Configuration
//**************************************************************************************************
const char* mqtt_server = "192.168.x.xxx";
const char* mqtt_username = "your_mqtt_username";
const char* mqtt_password = "your_mqtt_password";
const char* mqtt_topic = "your_mqtt_topic";
uint32_t INTERVAL = 300e6;  // 300 second timeout on the WiFi connection


//#define DEBUG  // prints WiFi connection info to serial, uncomment if you want WiFi messages
#ifdef DEBUG
#define DEBUG_PRINTLN(x)  Serial.println(x)
#define DEBUG_PRINT(x)  Serial.print(x)
#else
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT(x)
#endif

// #define USE_LED // uncomment to turn on the led when connecting
#define LED LED_BUILTIN

#define SERIAL_BAUD 115200
BME280I2C bme;    // Default : forced mode, standby time = 1000 ms
// Oversampling = pressure ×1, temperature ×1, humidity ×1, filter off,

// This structure is stored in RTC memory to save the WiFi state and reset count (number of Deep Sleeps),
// and it reconnects twice as fast as the first connection; it's used several places in this demo
struct nv_s {
  WiFiState wss; // core's WiFi save state
  struct {
    uint32_t crc32;
    uint32_t rstCount;  // stores the Deep Sleep reset count
    // you can add anything else here that you want to save, must be 4-byte aligned
  } rtcData;
};

static nv_s* nv = (nv_s*)RTC_USER_MEM; // user RTC RAM area


ADC_MODE(ADC_VCC);  // allows you to monitor the internal VCC level; it varies with WiFi load
// don't connect anything to the analog input pin(s)!

esp8266::polledTimeout::oneShotMs wifiTimeout(timeout);  // 30 second timeout on WiFi connection


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(75)
char msg[MSG_BUFFER_SIZE];
int value = 0;


void setup() {
  int duration;
  unsigned long start = millis();

  pinMode(LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
  Serial.begin(115200);
  Serial.println();
  Serial.print(F("\nReset reason = "));
  String resetCause = ESP.getResetReason();
  Serial.println(resetCause);

  setup_bme280();
  client.setServer(mqtt_server, 1883);

  initWiFi();
  if (WiFi.localIP()) {  // won't go into Automatic Sleep without an active WiFi connection
    Serial.println(F("Wifi connected. The amperage will drop in 7 seconds."));

    // Send MQTT
    if (!client.connected()) {
      reconnect();
    }
    client.loop();

  } else {
    Serial.println(F("no WiFi connection, transmission skipped"));
  }

  // Deep sleep mode for 30 seconds, the ESP8266 wakes up by itself when GPIO 16 (D0 in NodeMCU board) is connected to the RESET pin
  Serial.println("Going into deep sleep mode for 300 seconds (5 minutes)");
  duration = millis() - start;
  Serial.print("Run duration: ");
  Serial.println(duration);
  ESP.deepSleep(INTERVAL);
}


void loop() {

}


void updateRTCcrc() {  // updates the reset count CRC
  nv->rtcData.crc32 = crc32((uint8_t*) &nv->rtcData.rstCount, sizeof(nv->rtcData.rstCount));
}


void initWiFi() {
  #ifdef USE_LED
  digitalWrite(LED, LOW);  // give a visual indication that we're alive but busy with WiFi
  #endif
  
  uint32_t wifiBegin = millis();  // how long does it take to connect
  if ((crc32((uint8_t*) &nv->rtcData.rstCount + 1, sizeof(nv->wss)) && !WiFi.shutdownValidCRC(&nv->wss))) {
    // if good copy of wss, overwrite invalid (primary) copy
    memcpy((uint32_t*) &nv->wss, (uint32_t*) &nv->rtcData.rstCount + 1, sizeof(nv->wss));
  }
  if (WiFi.shutdownValidCRC(&nv->wss)) {  // if we have a valid WiFi saved state
    memcpy((uint32_t*) &nv->rtcData.rstCount + 1, (uint32_t*) &nv->wss, sizeof(nv->wss)); // save a copy of it
    Serial.println(F("resuming WiFi"));
  }
  if (!(WiFi.mode(WIFI_RESUME, &nv->wss))) {  // couldn't resume, or no valid saved WiFi state yet
    /* Explicitly set the ESP8266 as a WiFi-client (STAtion mode), otherwise by default it
      would try to act as both a client and an access-point and could cause network issues
      with other WiFi devices on your network. */
    WiFi.persistent(false);  // don't store the connection each time to save wear on the flash
    WiFi.mode(WIFI_STA);
    WiFi.setOutputPower(10);  // reduce RF output power, increase if it won't connect
    WiFi.config(staticIP, gateway, subnet);  // if using static IP, enter parameters at the top
    WiFi.begin(AP_SSID, AP_PASS);
    Serial.print(F("connecting to WiFi "));
    Serial.println(AP_SSID);
    DEBUG_PRINT(F("my MAC: "));
    DEBUG_PRINTLN(WiFi.macAddress());
  }
  wifiTimeout.reset(timeout);
  while (((!WiFi.localIP()) || (WiFi.status() != WL_CONNECTED)) && (!wifiTimeout)) {
    yield();
  }
  if ((WiFi.status() == WL_CONNECTED) && WiFi.localIP()) {
    DEBUG_PRINTLN(F("WiFi connected"));
    Serial.print(F("WiFi connect time = "));
    float reConn = (millis() - wifiBegin);
    Serial.printf("%1.2f seconds\n", reConn / 1000);
    DEBUG_PRINT(F("WiFi Gateway IP: "));
    DEBUG_PRINTLN(WiFi.gatewayIP());
    DEBUG_PRINT(F("my IP address: "));
    DEBUG_PRINTLN(WiFi.localIP());
  } else {
    Serial.println(F("WiFi timed out and didn't connect"));
  }
  WiFi.setAutoReconnect(true);
}


void setup_bme280() {
  Wire.begin();

  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch (bme.chipModel())
  {
    case BME280::ChipModel_BME280:
      Serial.println("Found BME280 sensor! Success.");
      break;
    case BME280::ChipModel_BMP280:
      Serial.println("Found BMP280 sensor! No Humidity available.");
      break;
    default:
      Serial.println("Found UNKNOWN sensor! Error!");
  }
}


void reconnect() {
  int tries = 0;
  // Loop until we're reconnected
  // After three retries skip transmission
  while (!client.connected() && tries < 3) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect( clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("mqtt connected");

      float temp(NAN), hum(NAN), pres(NAN);
      BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
      BME280::PresUnit presUnit(BME280::PresUnit_Pa);
      bme.read(pres, temp, hum, tempUnit, presUnit);

      float volts = ESP.getVcc() / 1000;
      Serial.printf("The internal VCC reads %1.2f volts\n", volts);

      snprintf (msg, MSG_BUFFER_SIZE, "{\"temp\" : \"%.1f\", \"hum\" : \"%.1f\", \"pres\" : \"%.1f\", \"volts\" : \"%.1f\"}", temp, hum, pres, volts);
      Serial.print("Publish message: ");

      Serial.println(msg);
      client.publish(mqtt_topic, msg);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      tries++;
    }
  }
}
