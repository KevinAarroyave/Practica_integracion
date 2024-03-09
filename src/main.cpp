#include <Arduino.h>
// Librería del profesor
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include "data.h"
#include "Settings.h"
// Incluir libreria del DHT11
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
// Incluir librería TFT_eSPI
#include <TFT_eSPI.h>
#include <SPI.h>
#include <Wire.h>
// Incluir libreria de Ubidots
#include "UbidotsEsp32Mqtt.h"
#include <PubSubClient.h>

// Definir pin a usar para capturar datos
#define DHTPIN 27
// Definir tipo de sensor
#define DHTTYPE DHT11 // DHT 11
// #define DHTTYPE    DHT22     // DHT 22 (AM2302)
// #define DHTTYPE    DHT21     // DHT 21 (AM2301)
const char *UBIDOTS_TOKEN = "BBUS-jJBSm7aUTrG4VUYbS1lvuOm6LAAdz1"; // Put here your Ubidots TOKEN
const char *PUBLISH_DEVICE_LABEL = "test";                         // Put here your Device label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL1 = "T";                         // Put here your Variable label to which data  will be published
const char *PUBLISH_VARIABLE_LABEL2 = "H";                         // Put here your Variable label to which data  will be published
const char *SUBSCRIBE_DEVICE_LABEL = "test";                       // Replace with the device label to subscribe to
const char *SUBSCRIBE_VARIABLE_LABEL = "SW1";                      // Replace with the variable label to subscribe to
const int PUBLISH_FREQUENCY = 5000;                                // Update rate in millisecondsx
unsigned long timer;
uint8_t LED = 26; // Pin used to read data from GPIO34 ADC_CH6.
bool POINT = false;
float T, H;

#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000 milis = 3s

// Invocar librería del Web server
WebServer server(80);
// Invocar librería de ubidots
Ubidots ubidots(UBIDOTS_TOKEN);
// Invocar librería DHT
DHT_Unified dht(DHTPIN, DHTTYPE);
// Invocar la librería del TTGO
TFT_eSPI tft = TFT_eSPI();
// definir delay par DHT11

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("fabio_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
    if ((char)payload[0] == '1')
    {
      digitalWrite(LED, HIGH);
      POINT = true;
    }
    else
    {
      digitalWrite(LED, LOW);
      POINT = false;
    }
  }
  Serial.println();
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int cnt = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    // Serial.print(".");
    if (cnt == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    cnt++;
    Serial.println("attempt # " + (String)cnt);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
  // Initialize device.
  pinMode(LED, OUTPUT);
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("°C"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("°C"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print(F("Sensor Type: "));
  Serial.println(sensor.name);
  Serial.print(F("Driver Ver:  "));
  Serial.println(sensor.version);
  Serial.print(F("Unique ID:   "));
  Serial.println(sensor.sensor_id);
  Serial.print(F("Max Value:   "));
  Serial.print(sensor.max_value);
  Serial.println(F("%"));
  Serial.print(F("Min Value:   "));
  Serial.print(sensor.min_value);
  Serial.println(F("%"));
  Serial.print(F("Resolution:  "));
  Serial.print(sensor.resolution);
  Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  tft.init();                                                                   // Inicializar la librería del display
  tft.setRotation(1);                                                           // Rotar los caracteres en el display
  ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
  timer = millis();
  tft.fillScreen(TFT_BLACK);   // Definir un color de fondo por la pantalla
  tft.setCursor(0, 0, 2);      // Ubicar el cursor en una posición inicial
  tft.setTextColor(TFT_WHITE); // Elegir un color para la letra
  tft.setTextSize(2);          // Definir un tamaño para la letra
  tft.println("Hola!");        // Imprimir texto
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    // Delay between measurements.
    // delay(delayMS);
    // Get temperature event and print its value.
    sensors_event_t event;

    // put your main code here, to run repeatedly:
    if (!ubidots.connected())
    {
      ubidots.reconnect();
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL); // Insert the device and variable's Labels, respectively
    }
    if ((millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {

      tft.fillScreen(TFT_BLACK);   // Definir un color de fondo por la pantalla
      tft.setCursor(0, 0, 2);      // Ubicar el cursor en una posición inicial
      tft.setTextColor(TFT_WHITE); // Elegir un color para la letra
      tft.setTextSize(2);          // Definir un tamaño para la letra
      tft.println("Hola!");        // Imprimir texto

      dht.temperature().getEvent(&event);
      if (isnan(event.temperature))
      {
        Serial.println(F("Error reading temperature!"));
      }
      else
      {
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
        tft.print("T = ");
        tft.print(event.temperature);
        tft.println(" °C");
        T = event.temperature;
      }
      // Get humidity event and print its value.
      dht.humidity().getEvent(&event);
      if (isnan(event.relative_humidity))
      {
        Serial.println(F("Error reading humidity!"));
      }
      else
      {
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
        tft.print("H = ");
        tft.print(event.relative_humidity);
        tft.println("%");
        H = event.relative_humidity;
      }

      ubidots.add(PUBLISH_VARIABLE_LABEL1, T); // Insert your variable Labels and the value to be sent
      ubidots.add(PUBLISH_VARIABLE_LABEL2, H); // Insert your variable Labels and the value to be sent
      ubidots.publish(PUBLISH_DEVICE_LABEL);
      ubidots.subscribeLastValue(SUBSCRIBE_DEVICE_LABEL, SUBSCRIBE_VARIABLE_LABEL);
      timer = millis();
      if (POINT)
      {
        tft.fillCircle(80, 15, 10, TFT_GREEN);
      }
      else
      {
        tft.fillCircle(80, 15, 10, TFT_DARKGREY);
      }
    }

    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}