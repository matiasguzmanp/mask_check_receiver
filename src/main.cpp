/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/
#include <Arduino.h>
#include "BLEDevice.h"
#include <Wire.h>

#include <vector>
#include <deque>
#include <algorithm>
#include <numeric>
//Default Temperature is in Celsius
//Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "ESP32-1"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
static BLEUUID bmeServiceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");

// BLE Characteristics
#ifdef temperatureCelsius
  //Temperature Celsius Characteristic
  static BLEUUID temperatureCharacteristicUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
#else
  //Temperature Fahrenheit Characteristic
  static BLEUUID temperatureCharacteristicUUID("f78ebbff-c8b7-4107-93de-889a6a06d408");
#endif

// Humidity Characteristic
static BLEUUID humidityCharacteristicUUID("e228c863-db2f-4b78-8bba-a25fb0a7f2e6");

// Pressure Characteristic
static BLEUUID pressureCharacteristicUUID("7cfee66f-6116-4cbf-8357-4b4ab2ed729a");

// Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

// Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

// Characteristics that we want to read
static BLERemoteCharacteristic* temperatureCharacteristic;
static BLERemoteCharacteristic* humidityCharacteristic;
static BLERemoteCharacteristic* pressureCharacteristic;

// Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};

// Variables to store temperature, humidity, and pressure
char temperatureChar[8];
char humidityChar[8];
char pressureChar[8];

// Flags to check whether new temperature, humidity, and pressure readings are available
boolean newTemperature = false;
boolean newHumidity = false;
boolean newPressure = false;

template <typename T>
class SensorBuffer {
public:
    SensorBuffer(size_t size) : maxSize(size) {}

    void addReading(T reading) {
        if (buffer.size() >= maxSize) {
            buffer.pop_front();
        }
        buffer.push_back(reading);
    }

    const std::deque<T>& getBuffer() const {
        return buffer;
    }
    T getMean() const {
        if (buffer.empty()) return T();
        T sum = std::accumulate(buffer.begin(), buffer.end(), T());
        return sum / buffer.size();
    }

    T getMax() const {
        if (buffer.empty()) return T();
        return *std::max_element(buffer.begin(), buffer.end());
    }

private:
    size_t maxSize;
    std::deque<T> buffer;
};

class SensorManager {
public:
    SensorManager(size_t bufferSize)
        : tempBuffer(bufferSize), humidityBuffer(bufferSize), pressureBuffer(bufferSize),
          pressureDerivativeBuffer(bufferSize), previousPressureReading(0.0), firstPressureReading(true) {}

    void addTemperatureReading(double reading) {
        tempBuffer.addReading(reading);
    }

    void addHumidityReading(double reading) {
        humidityBuffer.addReading(reading);
    }

    void addPressureReading(double reading) {
        if (firstPressureReading) {
            previousPressureReading = reading;
            firstPressureReading = false;
        }
        double pressureDerivative = reading - previousPressureReading;
        previousPressureReading = reading;

        pressureBuffer.addReading(reading);
        pressureDerivativeBuffer.addReading(fabs(pressureDerivative));
    }

    double getTemperatureMean() const {
        return tempBuffer.getMean();
    }

    double getTemperatureMax() const {
        return tempBuffer.getMax();
    }

    double getHumidityMean() const {
        return humidityBuffer.getMean();
    }

    double getHumidityMax() const {
        return humidityBuffer.getMax();
    }

    double getPressureMean() const {
        return pressureBuffer.getMean();
    }

    double getPressureMax() const {
        return pressureBuffer.getMax();
    }

    double getPressureDerivativeMean() const {
        return pressureDerivativeBuffer.getMean();
    }

    double getPressureDerivativeMax() const {
        return pressureDerivativeBuffer.getMax();
    }

    void printBuffers() const {
        Serial.print("Temperature buffer: ");
        printBuffer(tempBuffer.getBuffer());

        Serial.print("Humidity buffer: ");
        printBuffer(humidityBuffer.getBuffer());

        Serial.print("Pressure buffer: ");
        printBuffer(pressureBuffer.getBuffer());

        Serial.print("Pressure derivative buffer: ");
        printBuffer(pressureDerivativeBuffer.getBuffer());
    }

    void printMeanAndMax() const {
        Serial.print("Temp_mean: ");
        Serial.print(getTemperatureMean());
        Serial.print(" ");
        Serial.print("Temp_max: ");
        Serial.print(getTemperatureMax());
        Serial.print(" ");

        Serial.print("Hum_mean: ");
        Serial.print(getHumidityMean());
        Serial.print(" ");
        Serial.print("Hum_max: ");
        Serial.print(getHumidityMax());
        Serial.print(" ");

        Serial.print("Press_mean: ");
        Serial.print(getPressureMean());
        Serial.print(" ");
        Serial.print("Press_max: ");
        Serial.print(getPressureMax());
        Serial.print(" ");

        Serial.print("Press_dev_mean: ");
        Serial.print(getPressureDerivativeMean());
        Serial.print(" ");
        Serial.print("Press_dev_max: ");
        Serial.print(getPressureDerivativeMax());
        Serial.println();
    }

private:
    SensorBuffer<double> tempBuffer;
    SensorBuffer<double> humidityBuffer;
    SensorBuffer<double> pressureBuffer;
    SensorBuffer<double> pressureDerivativeBuffer;
    double previousPressureReading;
    bool firstPressureReading;

    void printBuffer(const std::deque<double>& buffer) const {
        for (const auto& reading : buffer) {
            Serial.print(reading);
            Serial.print(" ");
        }
        Serial.println();
    }
};


SensorManager sensorManager(10); // creates a sensor manager with a buffer size of 10

// When the BLE Server sends a new temperature reading with the notify property
static void temperatureNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                        uint8_t* pData, size_t length, bool isNotify) {
  // Copy temperature value
  strncpy(temperatureChar, (char*)pData, length);
  temperatureChar[length] = '\0'; // Null-terminate the string

  // Convert to double
  double temperatureValue = atof(temperatureChar);

  // Add to SensorManager
  sensorManager.addTemperatureReading(temperatureValue);

  newTemperature = true;
}

// When the BLE Server sends a new humidity reading with the notify property
static void humidityNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  // Copy humidity value
  strncpy(humidityChar, (char*)pData, length);
  humidityChar[length] = '\0'; // Null-terminate the string

  // Convert to double
  double humidityValue = atof(humidityChar);

  // Add to SensorManager
  sensorManager.addHumidityReading(humidityValue);

  newHumidity = true;
}

// When the BLE Server sends a new pressure reading with the notify property
static void pressureNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, 
                                    uint8_t* pData, size_t length, bool isNotify) {
  // Copy pressure value
  strncpy(pressureChar, (char*)pData, length);
  pressureChar[length] = '\0'; // Null-terminate the string

  // Convert to double
  double pressureValue = atof(pressureChar);

  // Add to SensorManager
  sensorManager.addPressureReading(pressureValue);


  newPressure = true;
}

// Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();

  // Connect to the remote BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return false;
  }

  // Obtain a reference to the characteristics in the service of the remote BLE server.
  temperatureCharacteristic = pRemoteService->getCharacteristic(temperatureCharacteristicUUID);
  humidityCharacteristic = pRemoteService->getCharacteristic(humidityCharacteristicUUID);
  pressureCharacteristic = pRemoteService->getCharacteristic(pressureCharacteristicUUID);

  if (temperatureCharacteristic == nullptr || humidityCharacteristic == nullptr || pressureCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");

  // Assign callback functions for the Characteristics
  temperatureCharacteristic->registerForNotify(temperatureNotifyCallback);
  humidityCharacteristic->registerForNotify(humidityNotifyCallback);
  pressureCharacteristic->registerForNotify(pressureNotifyCallback);
  return true;
}

// Callback function that gets called when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.getName() == bleServerName) { // Check if the name of the advertiser matches
      advertisedDevice.getScan()->stop(); // Scan can be stopped, we found what we are looking for
      pServerAddress = new BLEAddress(advertisedDevice.getAddress()); // Address of advertiser is the one we need
      doConnect = true; // Set indicator, stating that we are ready to connect
      Serial.println("Device found. Connecting!");
    }
  }
};

void printReadings() {
  Serial.print("Temperature:");
  Serial.print(temperatureChar);
  Serial.print(" C");

  Serial.print(" Humidity:");
  Serial.print(humidityChar); 
  Serial.print(" %");

  Serial.print(" Pressure: ");
  Serial.print(pressureChar);
  Serial.println(" hPa");
}


void setup() {
  // Start serial communication
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  // Init BLE device
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device. Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
}

void loop() {
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect. Now we connect to it. Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      // Activate the Notify property of each Characteristic
      temperatureCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      humidityCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      pressureCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      connected = true;
    } else {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
    }
    doConnect = false;
  }

  if (newTemperature && newHumidity && newPressure) {
    newTemperature = false;
    newHumidity = false;
    newPressure = false;
    //sensorManager.printBuffers();
    //printReadings();
    sensorManager.printMeanAndMax();
  }
  delay(1000); // Delay a second between loops.
}