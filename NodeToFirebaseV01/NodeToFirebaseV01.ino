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
#include <ESP8266WiFi.h>     // wifi module for esp8266
#include <time.h>            // time() ctime()
#include <sys/time.h>        // struct timeval
#include <coredecls.h>       // settimeofday_cb()
#include <SoftwareSerial.h>  // library for UART comm. between stm32 and nodeMCU


/* Config Variables for Wifi and Firebase Connection START */
#define WIFI_SSID "jeansPC"
#define WIFI_PASSWORD "qwertyui"

// The API key can be obtained from Firebase console > Project Overview > Project settings.
#define API_KEY "AIzaSyDKuF2rvUTePEPGIad4v3t68fzIRMX897E"

// User Email and password that already registerd or added in your project.
#define USER_EMAIL "paopeaw8246@gmail.com"
#define USER_PASSWORD "admin1234"

#define FIREBASE_PROJECT_ID "embed-2024"

/* Config Variables for Wifi and Firebase Connection END */


/* Config Variables for specific time zone START */
#define TZ +7     // (utc+) TZ in hours
#define DST_MN 0  // use 60mn for summer time in some countries
#define TZ_MN ((TZ)*60)
#define TZ_SEC ((TZ)*3600)
#define DST_SEC ((DST_MN)*60)

timeval cbtime;  // time set in callback
bool cbtime_set = false;
void time_is_set(void) {
  gettimeofday(&cbtime, NULL);
  cbtime_set = true;
}
/* Config Variables for specific time zone END */


/* Defining the UART Port START */
EspSoftwareSerial::UART stmPort;
char msg[150];  // A buffer of char for receiving data via UART
int i = 0;      // Index of current buffer char
/* Defining the UART Port END */


/* Defining variables to store our received value START */
double pressure_raw = 0;
double temperature_raw = 0;
double humidity_raw = 0;
int z_raw = 0;
/* Defining variables to store our received value END */

void processMessage(char* message);
void sendDataToFireStore(double pressure_val, double temperature_val, double humidity_val, int z_val);
void printResult(AsyncResult& aResult);

DefaultNetwork network;  // initilize with boolean parameter to enable/disable network reconnection
  
UserAuth user_auth(API_KEY, USER_EMAIL, USER_PASSWORD);

FirebaseApp app;  // defining Firebase as app

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

AsyncResult aResult_no_callback;  //callback after creating a new doc

void setup() {

  /* 
  Initializing the UART communication protocol port with 8N1 (8 bits data, no parity, 1 stop bit)
  using `D7 as Rx` and `D8 as Tx`
  */
  stmPort.begin(115200, EspSoftwareSerial::SWSERIAL_8N1, D7, D8, false, 150, 150);
  if (!stmPort) {  // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config");
    while (1) {  // Don't continue with invalid configuration
      delay(1000);
    }
  }

  Serial.begin(115200);

  /* Calibrate the local time by pulling the UNIX time from the network after connecting to the internet */
  settimeofday_cb(time_is_set);
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");


  /* Connect to Wifi START */
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\nConnecting to Wi-Fi\n");
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
  /*
  - Continuously listen for transmission from stm32.
  - Process the received data
  - Send the Data to Firestore
  */

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







void processMessage(char* message) {
  /*
  This function process the received message from UART.
  Then, call sendDataToFireStore() to send Data to Firestore.
  */

  //decompose the string into each variables
  sscanf(message, "s%lf,%lf,%lf,%d", &pressure_raw, &temperature_raw, &humidity_raw, &z_raw);

  Serial.print("Received Numbers: \nP:");
  Serial.print(pressure_raw);
  Serial.print(", T:");
  Serial.print(temperature_raw);
  Serial.print(", H:");
  Serial.print(humidity_raw);
  Serial.print(", Z:");
  Serial.print(z_raw);
  Serial.print("\n");

  //send Data to Firestore.
  sendDataToFireStore(pressure_raw, temperature_raw, humidity_raw, z_raw);
}

void sendDataToFireStore(double pressure_val, double temperature_val, double humidity_val, int z_val) {
  /*
  This function put sensor data into a desired format.
  Then create a document for Firestore at a specified path.
  */

  //check if FirebaseApp is ready or not
  if (app.ready()) {

    String DOCUMENT_PATH = "sensor_data/d" + String(random(30000));

    /* format the sensor data */
    Values::DoubleValue pressure(pressure_val);
    Values::IntegerValue date(time(nullptr));
    Values::DoubleValue temperature(temperature_val);
    Values::DoubleValue humidity(humidity_val);
    Values::IntegerValue z(z_val);

    /* Creating a document and push to Firestore*/
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
  /*
  Callback after Creating the Document.
  Displaying the status of the Created Document.
  */
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
