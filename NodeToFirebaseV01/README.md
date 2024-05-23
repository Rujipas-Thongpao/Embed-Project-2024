# Arduino ESP8266 Firebase Data Logger

This project demonstrates how to set up an ESP8266 microcontroller to read data from a connected STM32 microcontroller via UART, process the received data, and send it to Google Firestore using Firebase. The setup also includes configuring WiFi, managing time zones, and handling the Firebase client.

## Requirements

- **Hardware**:
  - ESP8266 (e.g., NodeMCU)
  - STM32 microcontroller
  - USB cables and necessary connectors
  - Breadboard and jumper wires for connections

- **Software**:
  - [Arduino IDE](https://www.arduino.cc/en/software)
  - Arduino libraries:
    - ESP8266WiFi
    - FirebaseClient
    - SoftwareSerial

- **Firebase**:
  - Firebase account and project setup
  - API Key and authentication details from the Firebase console

## Project Setup

### Hardware Connections
![Untitled](https://prod-files-secure.s3.us-west-2.amazonaws.com/85d575b5-7476-4c27-a2aa-1fb751b9e9a4/8179e881-40d4-4945-af96-c33f7706cc5c/Untitled.png)
1. Connect the STM32 to the ESP8266 via UART.
2. Use `D7` as Rx and `D8` as Tx for the ESP8266.

### Code Explanation

#### Libraries and Definitions

```cpp
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <FirebaseClient.h>
#include <time.h>
#include <sys/time.h>
#include <coredecls.h>
#include <SoftwareSerial.h>
```

These include necessary libraries for WiFi, Firebase, time handling, and software serial communication.

#### WiFi and Firebase Configuration

```cpp
#define WIFI_SSID "your_ssid"
#define WIFI_PASSWORD "your_password"
#define API_KEY "your_api_key"
#define USER_EMAIL "your_email"
#define USER_PASSWORD "your_password"
#define FIREBASE_PROJECT_ID "your_project_id"
```

Replace these placeholder values with your actual WiFi credentials and Firebase configuration.

#### Time Zone Configuration

```cpp
#define TZ +7
#define DST_MN 0
#define TZ_MN ((TZ)*60)
#define TZ_SEC ((TZ)*3600)
#define DST_SEC ((DST_MN)*60)

timeval cbtime;
bool cbtime_set = false;
void time_is_set(void) {
  gettimeofday(&cbtime, NULL);
  cbtime_set = true;
}
```

Configure the time zone as needed.

#### UART Configuration

```cpp
EspSoftwareSerial::UART stmPort;
char msg[150];
int i = 0;
```

Define the UART port and buffer for communication with the STM32.

#### Variables for Sensor Data

```cpp
double pressure_raw = 0;
double temperature_raw = 0;
double humidity_raw = 0;
int z_raw = 0;
```

Variables to store received sensor data.

#### Initialization and Setup

```cpp
void setup() {
  stmPort.begin(115200, EspSoftwareSerial::SWSERIAL_8N1, D7, D8, false, 150, 150);
  if (!stmPort) {
    Serial.println("Invalid EspSoftwareSerial pin configuration, check config");
    while (1) {
      delay(1000);
    }
  }

  Serial.begin(115200);
  settimeofday_cb(time_is_set);
  configTime(TZ_SEC, DST_SEC, "pool.ntp.org");

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

  Firebase.printf("Firebase Client v%s\n", FIREBASE_CLIENT_VERSION);
  Serial.println("Initializing app...");

  ssl_client.setInsecure();
  ssl_client.setBufferSizes(4096, 1024);

  initializeApp(aClient, app, getAuth(user_auth), aResult_no_callback);
  app.getApp<Firestore::Documents>(Docs);
}
```

This function initializes UART communication, sets up WiFi, synchronizes time, and configures the Firebase client.

#### Main Loop

```cpp
void loop() {
  app.loop();
  Docs.loop();

  if (stmPort.available()) {
    Serial.println("starting transmission...");
    while (stmPort.available()) {
      char c = (char)stmPort.read();
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
  printResult(aResult_no_callback);
}
```

The main loop continuously listens for data from the STM32, processes it, and sends it to Firestore.

#### Process Message

```cpp
void processMessage(char* message) {
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

  sendDataToFireStore(pressure_raw, temperature_raw, humidity_raw, z_raw);
}
```

This function processes the received message and extracts sensor values.

#### Send Data to Firestore

```cpp
void sendDataToFireStore(double pressure_val, double temperature_val, double humidity_val, int z_val) {
  if (app.ready()) {
    String DOCUMENT_PATH = "sensor_data/d" + String(random(9000000));

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
```

This function formats and sends sensor data to Firestore.

#### Print Result

```cpp
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
```

This function prints the result of the Firestore document creation.

## Conclusion

This project provides a comprehensive setup for reading sensor data via UART, processing it, and sending it to Firestore using an ESP8266. Ensure you have the necessary hardware connections and correct configuration values for WiFi and Firebase. This setup can be extended or modified to suit different sensors and use cases.
