/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/
#include <Wire.h>
#include "MutichannelGasSensor.h"
#include <SPI.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "Adafruit_HDC1000.h"
#include <math.h>
#include "EmonLib.h"


Adafruit_HDC1000 hdc = Adafruit_HDC1000();
EnergyMonitor emon1;

#define LED_PIN  5

#define PRE_HEAT_TIME   1

const float reference_vol = 0.500;
unsigned char clear_num = 0; //when use lcd to display
float R = 0;
float voltage = 0;


// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define aqServiceUUID        0xCCC0
#define allUUID 0xCCC1
#define nh3UUID 0xCCCa
#define coUUID  0xCCC2
#define no2UUID 0xCCC3
#define c3h8UUID 0xCCC4
#define c4h10UUID 0xCCC5
#define ch4UUID 0xCCC6
#define h2UUID 0xCCC6
//
#define aqService2UUID 0xDDD0
#define c2h5ohUUID 0xDDD1
#define tempUUID  0xDDD2
#define humUUID 0xDDD3
//
#define aqService3UUID 0xEEE0
#define power1UUID 0xEEE1
#define power2UUID 0xEEE2

#define getServiceUUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define switchUUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define calibUUID "19b10001-e8f2-537e-4f6c-d104768a1215"


class switchCharacteristicWritten: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      
      String strValue = "";
      
      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++) {
            Serial.print(String(value[i]));
            strValue += value[i];
          }   
        Serial.println();
        Serial.println("*********");
        
        if ( strValue == "1" ) {
          timerHandler();
        }
        if ( strValue == "2" ) {
          timer2Handler();
        }
        if (strValue == "3" ) {
          timer3Handler();
        }
        if ( strValue == "4" ) {
          timer4Handler();
        }
      }
    }
};

//BLEDescriptor allDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor nh3Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor coDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor no2Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor c3h8Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor c4h10Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor ch4Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor h2Descriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor c2h5ohDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor tempDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor humDescriptor(BLEUUID((uint16_t)0x2901));


BLECharacteristic nh3Characteristic(BLEUUID((uint16_t)nh3UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic coCharacteristic(BLEUUID((uint16_t)coUUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic no2Characteristic(BLEUUID((uint16_t)no2UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic c3h8Characteristic(BLEUUID((uint16_t)c3h8UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic c4h10Characteristic(BLEUUID((uint16_t)ch4UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic ch4Characteristic(BLEUUID((uint16_t)ch4UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic h2Characteristic(BLEUUID((uint16_t)h2UUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic c2h5ohCharacteristic(BLEUUID((uint16_t)c2h5ohUUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic tempCharacteristic(BLEUUID((uint16_t)tempUUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic humCharacteristic(BLEUUID((uint16_t)humUUID), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic switchCharacteristic(switchUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic calibCharacteristic(calibUUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);


volatile bool readFromSensor = true;
volatile bool readFromSensor2 = false;
volatile bool readFromSensor3 = false;
volatile bool readFromSensor4 = false;

float lastTempReading;
float lastHumidityReading;




void setup() {

  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  Serial.println("Starting BLE work!");

  BLEDevice::init("AirQuality");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();

  // Create the BLE Service
  BLEService *pService = pServer->createService(BLEUUID((uint16_t)aqServiceUUID));
  Serial.println("Starting BLE work!");

  nh3Descriptor.setValue("NH3");
  coDescriptor.setValue("CO");
  no2Descriptor.setValue("NO2");
  c3h8Descriptor.setValue("C3H8");
  c4h10Descriptor.setValue("C4H10");
  ch4Descriptor.setValue("CH4");
  h2Descriptor.setValue("H2");
  c2h5ohDescriptor.setValue("C2H5OH");
  tempDescriptor.setValue("temp");
  humCharacteristic.setValue("hum");


  //allCharacteristic.addDescriptor(&allDescriptor);
  nh3Characteristic.addDescriptor(&nh3Descriptor);
  coCharacteristic.addDescriptor(&coDescriptor);
  no2Characteristic.addDescriptor(&no2Descriptor);
  c3h8Characteristic.addDescriptor(&c3h8Descriptor);
  c4h10Characteristic.addDescriptor(&c4h10Descriptor);
  ch4Characteristic.addDescriptor(&ch4Descriptor);
  h2Characteristic.addDescriptor(&h2Descriptor);
  c2h5ohCharacteristic.addDescriptor(&c2h5ohDescriptor);
  tempCharacteristic.addDescriptor(&tempDescriptor);
  humCharacteristic.addDescriptor(&humDescriptor);

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  nh3Characteristic.addDescriptor(new BLE2902());
  coCharacteristic.addDescriptor(new BLE2902());
  no2Characteristic.addDescriptor(new BLE2902());
  c3h8Characteristic.addDescriptor(new BLE2902());
  c4h10Characteristic.addDescriptor(new BLE2902());
  ch4Characteristic.addDescriptor(new BLE2902());
  h2Characteristic.addDescriptor(new BLE2902());
  c2h5ohCharacteristic.addDescriptor(new BLE2902());
  tempCharacteristic.addDescriptor(new BLE2902());
  humCharacteristic.addDescriptor(new BLE2902());
  switchCharacteristic.addDescriptor(new BLE2902());
  //calibCharacteristic.addDescriptor(new BLE2902());

  Serial.println("Starting BLE work!");

  //  pService->addCharacteristic(&allCharacteristic);
  pService->addCharacteristic(&nh3Characteristic);
  pService->addCharacteristic(&coCharacteristic);
  pService->addCharacteristic(&no2Characteristic);
  pService->addCharacteristic(&c3h8Characteristic);
  pService->addCharacteristic(&c4h10Characteristic);

  pService->addCharacteristic(&ch4Characteristic);
  pService->addCharacteristic(&h2Characteristic);
  pService->addCharacteristic(&c2h5ohCharacteristic);
  pService->addCharacteristic(&tempCharacteristic);
  pService->addCharacteristic(&humCharacteristic);

  pService->addCharacteristic(&switchCharacteristic);
  //pService->addCharacteristic(&calibCharacteristic);

  delay(60);

  Serial.println("Starting BLE work! callback ");
  switchCharacteristic.setCallbacks(new switchCharacteristicWritten());
  delay(60);

  pService->start();
  Serial.println("Starting BLE work! Start");
  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println(F("BLE AirQ Sensor"));
  Serial.println("power on!");
  digitalWrite(LED_PIN, LOW);
  Serial.println("power on, and pre-heat");
  gas.begin(0x04);//the default I2C address of the slave is 0x04
  Serial.println("power on! powerON");
  gas.powerOn();
  delay(60);
  hdc.begin();
}

void getallVal() {
  float t;
  float h;
  float a;
  String r;

  t = hdc.readTemperature();
  r += String(t) + ",";
  h = hdc.readHumidity();
  r += String(h) + ",";
  a = gas.measure_NH3();
  r += String(a) + ",";
  a = gas.measure_CO();
  r += String(a) + ",";
  a = gas.measure_NO2();
  r += String(a) + ",";
  a = gas.measure_C3H8();
  r += String(a) + ",";
  a = gas.measure_C4H10();
  r += String(a) + ",";
  a = gas.measure_CH4();
  r += String(a) + ",";
  a = gas.measure_H2();
  r += String(a) + ",";
  a = gas.measure_C2H5OH();
  r += String(a) + ",";
  Serial.print(r);
  //  char *b ;
  //  b  = string2char(r);
  //  allCharacteristic.setValue(b);
  //   char b[20] ;
  //  r.toCharArray(b, 20);
  //  allCharacteristic.setValue((unsigned char*)b);
}

void calibrate() {

  for (int i = 60 * PRE_HEAT_TIME; i >= 0; i--)
  {
    Serial.print(i / 60);
    Serial.print(":");
    Serial.println(i % 60);
    delay(1000);
  }

  Serial.println("Begin to calibrate...");
  gas.doCalibrate();
  Serial.println("Calibration ok");
  delay(60);
}

void setPowerVal() {

}

void setTempHumVal() {
  float t;
  float h;


  t = hdc.readTemperature();
  h = hdc.readHumidity();
  uint8_t tempData[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(t * 100);
  // set  LSB of characteristic
  tempData[0] = tempValue;
  // set MSB of characteristic
  tempData[1] = tempValue >> 8;
  tempCharacteristic.setValue(tempData, 2);
  tempValue = (uint16_t)(h * 100);
  // set  LSB of characteristic
  tempData[0] = tempValue;
  // set MSB of characteristic
  tempData[1] = tempValue >> 8;
  humCharacteristic.setValue(tempData, 2);
  tempCharacteristic.notify();
  humCharacteristic.notify();
  Serial.print("T: "); Serial.print(t); Serial.println();
  Serial.print("H: "); Serial.print(h); Serial.println();
}
void setNH3CharacteristicValue() {

  float v;
  v = gas.measure_NH3();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  nh3Characteristic.setValue(a, 2);
  nh3Characteristic.notify();
  Serial.print("NH3: "); Serial.print(v); Serial.println();
}

void setCOCharacteristicValue() {

  float v;
  v = gas.measure_CO();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  coCharacteristic.setValue(a, 2);
  coCharacteristic.notify();
  Serial.print("CO: "); Serial.print(v); Serial.println();
}

void setNO2CharacteristicValue() {

  float v;
  v = gas.measure_NO2();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  no2Characteristic.setValue(a, 2);
  no2Characteristic.notify();
  Serial.print("NO2: "); Serial.print(v); Serial.println();
}

void setC3H8CharacteristicValue() {

  float v;
  v = gas.measure_C3H8();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  c3h8Characteristic.setValue(a, 2);
  c3h8Characteristic.notify();
  Serial.print(F("C3H8: ")); Serial.print(v); Serial.println();
}

void setC4H10CharacteristicValue() {

  float v;
  v = gas.measure_C4H10();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  c4h10Characteristic.setValue(a, 2);
  c4h10Characteristic.notify();
  Serial.print("C4H10: "); Serial.print(v); Serial.println();
}

void setCH4CharacteristicValue() {

  float v;
  v = gas.measure_CH4();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  ch4Characteristic.setValue(a, 2);
  ch4Characteristic.notify();
  Serial.print("CH4: "); Serial.print(v); Serial.println();
}

void setH2CharacteristicValue() {

  float v;
  v = gas.measure_H2();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  h2Characteristic.setValue(a, 2);
  h2Characteristic.notify();
  Serial.print("H2: "); Serial.print(v); Serial.println();
}

void setC2H5OHCharacteristicValue() {

  float v;
  v = gas.measure_C2H5OH();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  c2h5ohCharacteristic.setValue(a, 2);
  c2h5ohCharacteristic.notify();
  Serial.print(F("C2H5OH: ")); Serial.print(v); Serial.println();
}

void setEMPTYCharacteristicValue() {
  float v;
  v = gas.measure_C2H5OH();
  uint8_t a[2];
  uint16_t tempValue;
  // multiply by 100 to get 2 digits mantissa and convert into uint16_t
  tempValue = (uint16_t)(v * 100);
  // set  LSB of characteristic
  a[0] = tempValue;
  // set MSB of characteristic
  a[1] = tempValue >> 8;
  c2h5ohCharacteristic.setValue(a, 2);
  c2h5ohCharacteristic.notify();
  Serial.print("C2H5OH: "); Serial.print(v); Serial.println();
}

boolean significantChange(float val1, float val2, float threshold) {
  return (abs(val1 - val2) >= threshold);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (readFromSensor) {
    digitalWrite(LED_PIN, HIGH);
    setNH3CharacteristicValue();
    setCOCharacteristicValue();
    setNO2CharacteristicValue();
    setC3H8CharacteristicValue();
    setC4H10CharacteristicValue();
    readFromSensor = false;
    delay(10000);
    digitalWrite(LED_PIN, LOW);
  }

  if (readFromSensor2) {
    digitalWrite(LED_PIN, HIGH);
    setCH4CharacteristicValue();
    setH2CharacteristicValue();
    setC2H5OHCharacteristicValue();
    setTempHumVal();
    readFromSensor2 = false;
    delay(10000);
    digitalWrite(LED_PIN, LOW);
  }
  if (readFromSensor3) {
    digitalWrite(LED_PIN, HIGH);
    setPowerVal();
    readFromSensor3 = false;
    digitalWrite(LED_PIN, LOW);
  }
  if (readFromSensor4) {
    calibCharacteristic.setValue("0");
    digitalWrite(LED_PIN, HIGH);
    calibrate();
    readFromSensor4 = false;
    digitalWrite(LED_PIN, LOW);
    calibCharacteristic.setValue("1");
  }
  delay(60);

}


//void switchCharacteristicWritten() {
//  // central wrote new value to characteristic, update LED
//  Serial.print("Characteristic event, writen: ");
//  std::string value = switchCharacteristic.getValue();
//
//  if (value.length() > 0) {
//    Serial.println("*********");
//    Serial.print("New value: ");
//    for (int i = 0; i < value.length(); i++)
//      Serial.print(value[i]);
//
//    Serial.println();
//    Serial.println("*********");
//  }
//  if ( switchCharacteristic.getValue() == "1" ) {
//    timerHandler();
//  }
//  if ( switchCharacteristic.getValue() == "2" ) {
//    timer2Handler();
//  }
//  if ( switchCharacteristic.getValue() == "3" ) {
//    timer3Handler();
//  }
//  if ( switchCharacteristic.getValue() == "4" ) {
//    timer4Handler();
//  }
//}

void timerHandler() {
  readFromSensor = true;
}

void timer2Handler() {
  readFromSensor2 = true;
}

void timer3Handler() {
  readFromSensor3 = true;
}
void timer4Handler() {
  readFromSensor4 = true;
}
char* string2char(String command) {
  if (command.length() != 0) {
    char *p = const_cast<char*>(command.c_str());
    return p;
  }
}
