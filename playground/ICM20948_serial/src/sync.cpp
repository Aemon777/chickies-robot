/*********************************
//Reads in IMU data from ICM20948 and transmits it over serial
//Designed to run on Arduino Nano Every
//Author: Josh Blackburn
//Date Created: 2/23/2022
//Last Updated: 3/18/2022
*********************************/
#include <Arduino.h>
#include <Wire.h>
#include <ICM20948_WE.h>
#define ICM20948_ADDR 0x68
#define loop_hz 100

#define Serial SerialUSB

ICM20948_WE myIMU = ICM20948_WE(ICM20948_ADDR);
unsigned long previousLoopTime = 0;

xyzFloat g2mpsps(xyzFloat acceleration) {
  xyzFloat temporary;
  temporary.x = acceleration.x*9.81;
  temporary.y = acceleration.y*9.81;
  temporary.z = acceleration.z*9.81;
  return temporary;
}
xyzFloat dps2rps(xyzFloat angular_velocity) {
  xyzFloat temporary;
  temporary.x = angular_velocity.x/57.3;
  temporary.y = angular_velocity.y/57.3;
  temporary.z = angular_velocity.z/57.3;
  return temporary;
}

bool parseSerial() {
  char incomingString[16];
  byte i = 0;
  while(Serial.available() > 0) {
    incomingString[i] = Serial.read();
    i++;
    delayMicroseconds(50);
  }//Q,**
  //Serial1.println(incomingString);
  char* pch;
  char* chrt[4];
  int count = 0;
  pch = strtok (incomingString,",");
  while (pch != NULL)  {
    chrt[count] = pch;
    pch = strtok (NULL, ",");
    count++;
  }
  char indicator = chrt[0][0];
  if(indicator == 'Q') {
    return true;
  }
  else {
    return false;
  }
}//*/

void setup() {
  Wire.begin();
  Serial.begin(115200);
  while(!Serial);
  
  if(!myIMU.init()){
    Serial.println("ICM20948 does not respond");
  }

  if(!myIMU.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
      
  /* The starting point, if you position the ICM20948 flat, is not necessarily 0g/0g/1g for x/y/z. 
   * The autoOffset function measures offset. It assumes your ICM20948 is positioned flat with its 
   * x,y-plane. The more you deviate from this, the less accurate will be your results.
   * It overwrites the zero points of setAccOffsets, but keeps the correction of the slope.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called after setAccOffsets but before other settings since it will 
   * overwrite settings!
   */
  delay(100);
  myIMU.autoOffsets();
  
  /*  ICM20948_ACC_RANGE_2G      2 g   (default)
   *  ICM20948_ACC_RANGE_4G      4 g
   *  ICM20948_ACC_RANGE_8G      8 g   
   *  ICM20948_ACC_RANGE_16G    16 g
   */
  myIMU.setAccRange(ICM20948_ACC_RANGE_2G);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off.  
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              246.0               1125/(1+ASRD) 
   *    1              246.0               1125/(1+ASRD)
   *    2              111.4               1125/(1+ASRD)
   *    3               50.4               1125/(1+ASRD)
   *    4               23.9               1125/(1+ASRD)
   *    5               11.5               1125/(1+ASRD)
   *    6                5.7               1125/(1+ASRD) 
   *    7              473.0               1125/(1+ASRD)
   *    OFF           1209.0               4500
   *    
   *    ASRD = Accelerometer Sample Rate Divider (0...4095)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setAccDLPF(ICM20948_DLPF_6);    
  
  /*  ICM20948_GYRO_RANGE_250       250 degrees per second (default)
   *  ICM20948_GYRO_RANGE_500       500 degrees per second
   *  ICM20948_GYRO_RANGE_1000     1000 degrees per second
   *  ICM20948_GYRO_RANGE_2000     2000 degrees per second
   */
  myIMU.setGyrRange(ICM20948_GYRO_RANGE_250);
  
  /*  Choose a level for the Digital Low Pass Filter or switch it off. 
   *  ICM20948_DLPF_0, ICM20948_DLPF_2, ...... ICM20948_DLPF_7, ICM20948_DLPF_OFF 
   *  
   *  DLPF       3dB Bandwidth [Hz]      Output Rate [Hz]
   *    0              196.6               1125/(1+GSRD) 
   *    1              151.8               1125/(1+GSRD)
   *    2              119.5               1125/(1+GSRD)
   *    3               51.2               1125/(1+GSRD)
   *    4               23.9               1125/(1+GSRD)
   *    5               11.6               1125/(1+GSRD)
   *    6                5.7               1125/(1+GSRD) 
   *    7              361.4               1125/(1+GSRD)
   *    OFF          12106.0               9000
   *    
   *    GSRD = Gyroscope Sample Rate Divider (0...255)
   *    You achieve lowest noise using level 6  
   */
  myIMU.setGyrDLPF(ICM20948_DLPF_6);  

  /* You can set the following modes for the magnetometer:
   * AK09916_PWR_DOWN          Power down to save energy
   * AK09916_TRIGGER_MODE      Measurements on request, a measurement is triggered by 
   *                           calling setMagOpMode(AK09916_TRIGGER_MODE)
   * AK09916_CONT_MODE_10HZ    Continuous measurements, 10 Hz rate
   * AK09916_CONT_MODE_20HZ    Continuous measurements, 20 Hz rate
   * AK09916_CONT_MODE_50HZ    Continuous measurements, 50 Hz rate
   * AK09916_CONT_MODE_100HZ   Continuous measurements, 100 Hz rate (default)
   */
  myIMU.setMagOpMode(AK09916_CONT_MODE_100HZ);
}

void loop() {
  while(!Serial.available()) {
    if(millis() - previousLoopTime > (1000/loop_hz)) {
      previousLoopTime = millis();
      //update the values stored in the arduino with the latest values stored in the sensor's buffers
      myIMU.readSensor();
    }
  }
  if(parseSerial()) {
    //xyzFloat is a data struct containing three floats, one for each x,y,z
    xyzFloat gs = myIMU.getGValues();//g
    xyzFloat dps = myIMU.getGyrValues();//degrees/second
    xyzFloat magValues = myIMU.getMagValues();//uTeslas
    //acceleration and angular velocity come in g and degrees/second units, we need m/s^2 and radians/second
    xyzFloat accel = g2mpsps(gs);//m/s^2
    xyzFloat angvel = dps2rps(dps);//radians/second
    //transmit everything in format "I,aX,aY,aZ,gX,gY,gZ,mX,mY,mZ,**" over serial
    //where aX is acceleration in X, gX is angular velocity in X, and mX is magnetic field strength in X
    Serial.print("I,");
    Serial.print(accel.x);
    Serial.print(",");
    Serial.print(accel.y);
    Serial.print(",");
    Serial.print(accel.z);
    Serial.print(",");
    Serial.print(angvel.x);
    Serial.print(",");
    Serial.print(angvel.y);
    Serial.print(",");
    Serial.print(angvel.z);
    Serial.print(",");
    Serial.print(magValues.x);
    Serial.print(",");
    Serial.print(magValues.y);
    Serial.print(",");
    Serial.print(magValues.z);
    Serial.println(",**");//*/
  }
}