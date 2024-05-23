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

#define TZ -8      // (utc+) TZ in hours
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


String DOCUMENT_PATH = "sensor_data/d" + String(random(30000));

/* defining the UART Port START */
#include <SoftwareSerial.h>
EspSoftwareSerial::UART stmPort;
char msg[150];
int i = 0;

void processMessage(char* message) {
  // uncomment to debug (print data stored in message)
  Serial.print("Received Message: ");
  Serial.println(message);

  float dustval, latval, lonval;
  unsigned char latdir, londir;
  sscanf(message, "%f,%f,%c,%f,%c", &dustval, &latval, &latdir, &lonval, &londir);
  // uncomment to debug (print data from message after factoring into variables)
  // Serial.print("Received Numbers: ");
  // Serial.print(dustval);
  // Serial.print(", ");
  // Serial.print(latval);
  // Serial.print(", ");
  // Serial.print(latdir);
  // Serial.print(", ");
  // Serial.print(lonval);
  // Serial.print(", ");
  // Serial.println(londir);
  sendDataToFireStore(DOCUMENT_PATH, 10.5, 20.1, 30.5, 1);
}

/* defining the UART Port END */

void printResult(AsyncResult& aResult);

String getTimestampString(uint64_t sec, uint32_t nano);

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

  Serial.begin(115200);

  settimeofday_cb(time_is_set);
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");


  /* Connect to Wifi START */

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("\nConnecting to Wi-Fi\n");
  unsigned long ms = millis();
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

uint32_t lastSend = 0;

void loop() {

  // The async task handler should run inside the main loop
  // without blocking delay or bypassing with millis code blocks.

  app.loop();

  Docs.loop();


  if (app.ready() && millis() - lastSend >= 30000 || lastSend == 0) {
    lastSend = millis();

    sendDataToFireStore(DOCUMENT_PATH, 10.5, 20.1, 30.5, 1);
  }

  //Asycn callback printing for the sended Data status
  printResult(aResult_no_callback);
}

void sendDataToFireStore(String documentPath_val, double pressure_val, double humidity_val, double temperature_val, int z_val) {

  Values::DoubleValue pressure(pressure_val);
  Values::DoubleValue humidity(humidity_val);
  Values::DoubleValue temperature(temperature_val);
  Values::IntegerValue z(z_val);
  Values::IntegerValue date(time(nullptr));

  Document<Values::Value> doc("pressure", Values::Value(pressure));
  doc.add("date", Values::Value(date));
  doc.add("humidity", Values::Value(humidity));
  doc.add("temperature", Values::Value(temperature));
  doc.add("z", Values::Value(z));

  Serial.println("Create document... ");

  Docs.createDocument(aClient, Firestore::Parent(FIREBASE_PROJECT_ID), documentPath_val, DocumentMask(), doc, aResult_no_callback);
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

String getTimestampString(uint64_t sec, uint32_t nano) {
  if (sec > 0x3afff4417f)
    sec = 0x3afff4417f;

  if (nano > 0x3b9ac9ff)
    nano = 0x3b9ac9ff;

  time_t now;
  struct tm ts;
  char buf[80];
  now = sec;
  ts = *localtime(&now);

  String format = "%Y-%m-%dT%H:%M:%S";

  if (nano > 0) {
    String fraction = String(double(nano) / 1000000000.0f, 9);
    fraction.remove(0, 1);
    format += fraction;
  }
  format += "Z";

  strftime(buf, sizeof(buf), format.c_str(), &ts);
  return buf;
}


/**
 * SYNTAX:
 *
 * Firestore::Documents::createDocument(<AsyncClient>, <Firestore::Parent>, <documentPath>, <DocumentMask>, <Document>, <AsyncResult>);
 *
 * Firestore::Documents::createDocument(<AsyncClient>, <Firestore::Parent>, <collectionId>, <documentId>, <DocumentMask>, <Document>, <AsyncResult>);
 *
 * <AsyncClient> - The async client.
 * <Firestore::Parent> - The Firestore::Parent object included project Id and database Id in its constructor.
 * <documentPath> - The relative path of document to create in the collection.
 * <DocumentMask> - The fields to return. If not set, returns all fields. Use comma (,) to separate between the field names.
 * <collectionId> - The document id of document to be created.
 * <documentId> - The relative path of document collection id to create the document.
 * <Document> - The Firestore document.
 * <AsyncResult> - The async result (AsyncResult).
 *
 * The Firebase project Id should be only the name without the firebaseio.com.
 * The Firestore database id should be (default) or empty "".
 *
 * The complete usage guidelines, please visit https://github.com/mobizt/FirebaseClient
 */
