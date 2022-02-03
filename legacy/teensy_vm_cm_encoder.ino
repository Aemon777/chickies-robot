#include <Wire.h>
#include <Encoder.h>
#include <Adafruit_ADS1X15.h>

Encoder knobLeft(5, 6);
Encoder knobRight(7, 8);

static unsigned long previousPow = 0;
static unsigned long previousEnc = 0;
double positionLeft  = 0;
double positionRight = 0;

//1 second
const long second = 1000;

Adafruit_ADS1115 ads;
void setup() {
  Serial.begin(115200);
  setupADC();
}

void loop() {
  unsigned long current = millis();
  if(current - previousPow > 100) {
    previousPow = current;
    measurePower();
  }
  if(current - previousEnc > second){
    previousEnc = current;
    trackEncoders();
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

void trackEncoders(){  
  double newLeft, newRight;
  //divide by 215 for converting to centimeters
  newLeft = knobLeft.read() / 215.00;
  newRight = knobRight.read() / 215.00;
  if (newLeft != positionLeft || newRight != positionRight) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.print(", Right = ");
    Serial.print(newRight);
    Serial.println();
    positionLeft = newLeft;
    positionRight = newRight;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset both knobs to zero");
    knobLeft.write(0);
    knobRight.write(0);
  }
}
