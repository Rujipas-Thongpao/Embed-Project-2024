#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W) || defined(ARDUINO_GIGA)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#elif __has_include(<WiFiNINA.h>) || defined(ARDUINO_NANO_RP2040_CONNECT)
#include <WiFiNINA.h>
#elif __has_include(<WiFi101.h>)
#include <WiFi101.h>
#elif __has_include(<WiFiS3.h>) || defined(ARDUINO_UNOWIFIR4)
#include <WiFiS3.h>
#elif __has_include(<WiFiC3.h>) || defined(ARDUINO_PORTENTA_C33)
#include <WiFiC3.h>
#elif __has_include(<WiFi.h>)
#include <WiFi.h>
#endif


#include <FirebaseClient.h>  // library for sending data to FireStore
#include <ESP8266WiFi.h>     // wifi module
#include <time.h>            // time() ctime()
#include <sys/time.h>        // struct timeval
#include <coredecls.h>       // settimeofday_cb()

#define TZ +7      // (utc+) TZ in hours
#define DST_MN 60  // use 60mn for summer time in some countries
#define TZ_MN ((TZ)*60)
#define TZ_SEC ((TZ)*3600)
#define DST_SEC ((DST_MN)*60)

timeval cbtime;  // time set in callback

bool cbtime_set = false;

void time_is_set(void) {
  gettimeofday(&cbtime, NULL);
  cbtime_set = true;
}

#define WIFI_SSID "jeansPC"
#define WIFI_PASSWORD "qwertyui"

// The API key can be obtained from Firebase console > Project Overview > Project settings.
#define API_KEY "AIzaSyDKuF2rvUTePEPGIad4v3t68fzIRMX897E"

// User Email and password that already registerd or added in your project.
#define USER_EMAIL "paopeaw8246@gmail.com"
#define USER_PASSWORD "admin1234"

#define FIREBASE_PROJECT_ID "embed-2024"

/* defining the UART Port START */
#include <SoftwareSerial.h>
EspSoftwareSerial::UART stmPort;
char msg[150];
int i = 0;

double pressure_raw = 0;
double temperature_raw = 0;
double humidity_raw = 0;
int z_raw = 0;

void processMessage(char* message) {
  // uncomment to debug (print data stored in message)
  // Serial.print("Received Message: ");
  // Serial.println(message);

  sscanf(message, "s%lf,%lf,%lf,%d", &pressure_raw, &temperature_raw, &humidity_raw, &z_raw);
  // uncomment to debug (print data from message after factoring into variables)
  Serial.print("Received Numbers: \nP:");
  Serial.print(pressure_raw);
  Serial.print(", T:");
  Serial.print(temperature_raw);
  Serial.print(", H:");
  Serial.print(humidity_raw);
  Serial.print(", Z:");
  Serial.print(z_raw);
  Serial.print("\n");

  sendDataToFireStore(pressure_raw, temperature_raw, humidity_raw, z_raw);
}

/* defining the UART Port END */

void printResult(AsyncResult& aResult);

DefaultNetwork network;  // initilize with boolean parameter to enable/disable network reconnection

UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);

FirebaseApp app;

#if defined(ESP32) || defined(ESP8266) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFiClientSecure.h>
WiFiClientSecure ssl_client;
#elif defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_UNOWIFIR4) || defined(ARDUINO_GIGA) || defined(ARDUINO_PORTENTA_C33) || defined(ARDUINO_NANO_RP2040_CONNECT)
#include <WiFiSSLClient.h>
WiFiSSLClient ssl_client;
#endif

using AsyncClient = AsyncClientClass;

AsyncClient aClient(ssl_client, getNetwork(network));

Firestore::Documents Docs;

AsyncResult aResult_no_callback;

bool taskCompleted = false;

int cnt = 0;

void setup() {
  stmPort.begin(115200, EspSoftwareSerial::SWSERIAL_8N1, D7, D8, false, 150, 150);
  if (!stmPort) {  // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config");
    while (1) {  // Don't continue with invalid configuration
      delay(1000);
    }
  }
  Serial.begin(115200);

  settimeofday_cb(time_is_set);
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");


  /* Connect to Wifi START */

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\nConnecting to Wi-Fi\n");
  // unsigned long ms = millis();
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  /* Connect to Wifi END */

  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
  Serial.println("Initializing app...");

#if defined(ESP32) || defined(ESP8266) || defined(PICO_RP2040)
  ssl_client.setInsecure();
#if defined(ESP8266)
  ssl_client.setBufferSizes(4096, 1024);
#endif
#endif

  initializeApp(aClient, app, getAuth(user_auth), aResult_no_callback);

  app.getApp<Firestore::Documents>(Docs);
}

// uint32_t lastSend = 0;

void loop() {

  // The async task handler should run inside the main loop
  // without blocking delay or bypassing with millis code blocks.

  app.loop();

  Docs.loop();

  if (stmPort.available()) {
    Serial.println("starting transmissiom...");
    while (stmPort.available()) {
      char c = (char)stmPort.read();
      //uncomment to debug (print data from stm32 board)
      // Serial.print(c);

      msg[i++] = c;
      if (i == 150) {
        Serial.println("End of Buffer Reached...Current Receiving loop terminated");
        i = 0;
        break;
      }
      if (c == '\n') {
        Serial.println("Message received successfully (endline found.)");
        msg[i] = '\0';
        processMessage(msg);
        i = 0;
      }
    }
  }

  //Asycn callback printing for the sended Data status
  printResult(aResult_no_callback);
}

void sendDataToFireStore(double pressure_val, double temperature_val, double humidity_val, int z_val) {
  if (app.ready()) {

    String DOCUMENT_PATH = "sensor_data/d" + String(random(30000));

    Values::DoubleValue pressure(pressure_val);
    Values::IntegerValue date(time(nullptr));
    Values::DoubleValue temperature(temperature_val);
    Values::DoubleValue humidity(humidity_val);
    Values::IntegerValue z(z_val);

    Document<Values::Value> doc("pressure", Values::Value(pressure));
    doc.add("date", Values::Value(date));
    doc.add("temperature", Values::Value(temperature));
    doc.add("humidity", Values::Value(humidity));
    doc.add("z", Values::Value(z));

    Serial.println("Create document... ");
    Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), DOCUMENT_PATH, DocumentMask(), doc, aResult_no_callback);
  }
}

void printResult(AsyncResult& aResult) {
  if (aResult.isEvent()) {
    Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.appEvent().message().c_str(), aResult.appEvent().code());
  }

  if (aResult.isDebug()) {
    Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
  }

  if (aResult.isError()) {
    Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
  }

  if (aResult.available()) {
    Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
  }
}
