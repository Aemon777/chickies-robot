#include <Wire.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
void setup() {
  Serial.begin(115200);
  setupADC();
}


void loop() {
  unsigned long current = millis();
  static unsigned long previous = 0;
  if(current - previous > 100) {
    previous = current;
    measurePower();
  }
}
void setupADC() {
  Wire.begin(); // join i2c bus (address optional for master)
  ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V  1 bit = 1mV      0.0625mV
  if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}
void measurePower() {
  int16_t adc0, adc1;
  float voltage, current;
  static float energy = 0.00;
  adc0 = ads.readADC_SingleEnded(0);
  adc1 = ads.readADC_SingleEnded(1);
  voltage = ads.computeVolts(adc0);
  current = ads.computeVolts(adc1);
  voltage *= 10;
  current = current * 20 - 0.7;
  Wire.beginTransmission(0x59); // transmit to device #4
  char str[5];
  dtostrf(voltage,5,2,str);
  Wire.write(str);        // sends five bytes
  //Wire.write(x);              // sends one byte  
  Wire.endTransmission();    // stop transmitting
  String data = "V,";
  data += voltage;
  data += ",C,";
  data += current;
  data += ",P,";
  float power = voltage * current;
  data += power;
  data += ",E,";
  energy += power/36;
  data += energy/1000;
  Serial.println(data);
}
