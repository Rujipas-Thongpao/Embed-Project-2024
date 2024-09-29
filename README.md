![image](https://github.com/user-attachments/assets/882c24e4-308d-4591-bd62-d5b2a68c2c85)
# Project Description
This project is an embedded system for weather forecasting using the STM32 microcontroller as the main board. It utilizes two sensors, DHT22 (for temperature and humidity) and BMP280 (for pressure), to collect environmental data. The data is sent to a Firebase Firestore database via the NodeMCU (ESP8266) module. The system uses the Zambretti Algorithm for forecasting weather conditions, and the data visualization is implemented using Vue.js for real-time weather monitoring.

# Components Used:
- STM32 Microcontroller: Main processing unit.
- DHT22 Sensor: Measures temperature and humidity.
- BMP280 Sensor: Measures atmospheric pressure.
- NodeMCU (ESP8266): Handles Wi-Fi connectivity and communication with Firebase Firestore.
- Firebase Firestore: Cloud database for storing weather data.
- Vue.js: Frontend framework used for data visualization.
- Zambretti Algorithm: Algorithm used for weather forecasting based on sensor readings.
# Functionality:
- STM32 collects data from DHT22 and BMP280 sensors.
- NodeMCU sends this data to Firebase Firestore.
- Weather forecasts are calculated using the Zambretti Algorithm.
- Data is visualized in real-time using a Vue.js web application.
