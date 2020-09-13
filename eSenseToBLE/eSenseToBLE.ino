/*
 * Author: Henry Ugochukwu Odoemelem
 * 
 * Date: 12-08-2019
 * 
 * Description: ESP32 code to receive IMU data from eSense 
 * via bluetooth to control a robot arm.
 * 
 * Paper:Using the eSense Wearable Earbud as a Light-Weight Robot Arm Controller
 * https://doi.org/10.1145/3345615.3361138
 * 
 */

#include "BLEDevice.h"
#include <math.h>
#include <ESP32Servo.h>

#define   M_PI   3.14159265358979323846
using std::string;
// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ff06-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID    charUUID(BLEUUID((uint16_t)0xFF08));
static BLEUUID    sampleUUID(BLEUUID((uint16_t)0xFF07));
static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;

bool isCalibrated = 0;
double gyrox;
double gyroy;
double gyroz;
double accx;
double accy;
double accz;
double tempx,tempy;
double  angle_roll_gyro;
double angle_pitch_gyro;
bool  set_init_angles = false;
double to;
double f;
double rc;
double    angle_pitchz;
double      angle_pitch_accz;
double angle_pitchzOffset;
double    angle_rollx;
double   angle_roll_accx;
double angle_rollxOffset;
double pitch;
double roll;
float alpha;
int counter = 1;

 create servo object to control a servo
Servo base; 
Servo arm1; 
Servo arm2;

uint8_t IMUdata[16];
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicS;


static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

  //Serial.println("IMU Data: ");
  for (int i = 0; i < length; i++) {
    //Serial.print((signed char)pData[i],HEX); Serial.print(" ");
    IMUdata[i] = pData[i];
  }
  // Serial.print("\n");
  gyrox = (short) ((IMUdata[4] * 256) + IMUdata[5]);//(int16_t) ((IMUdata[4] << 8) | (IMUdata[5]));
  gyroy =  (short) ((IMUdata[6] * 256) + IMUdata[7]);
  gyroz = (short) ((IMUdata[8] * 256) + IMUdata[9]);


  accx = (short) ((IMUdata[10] * 256) + IMUdata[11]);//(int16_t) ((IMUdata[10] << 8) | (IMUdata[11]));
  accy =  (short) ((IMUdata[12] * 256) + IMUdata[13]);
  accz = (short) ((IMUdata[14] * 256) + IMUdata[15]);

  accx = (double)accx / 8192.0;
  accy = (double)accy / 8192.0;
  accz = (double)accz / 8192.0;

  //transform values from worn orientation to normal orientation
  tempx=accx;//temp. holders
  tempy=accy;
  accx=tempy;
  accy=-tempx;

  //covert gyro value to angle
  angle_roll_gyro = (double)gyroy / 65.5;  
  angle_pitch_gyro = (double)gyroz / 65.5;
}

bool connectToServer(BLEAddress pAddress) {
  Serial.print("Forming a connection to ");
  Serial.println(pAddress.toString().c_str());

  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    return false;
  }
  Serial.println(" - Found our characteristic");


  pRemoteCharacteristic->registerForNotify(notifyCallback);// register imu for notification

  pRemoteCharacteristicS = pRemoteService->getCharacteristic(sampleUUID);


  return true;
}
/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    /**
        Called for each advertising BLE server.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      Serial.print("BLE Advertised Device found: ");
      Serial.println(advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

        //
        Serial.print("Found our device!  address: ");
        advertisedDevice.getScan()->stop();

        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        doConnect = true;

      } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks



void setup() {
  Serial.begin(9600);
  Serial.println("Starting ESP32 BLE Client application...");

  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);


  base.setPeriodHertz(50);
  arm1.setPeriodHertz(50);
  arm2.setPeriodHertz(50);
  
  //9,10,11
  base.attach(2);  // attaches the servo on pin 9 to the servo object
  arm1.attach(18);  // attaches the servo on pin 10 to the servo object
  arm2.attach(5);  // attaches the servo on pin 11 to the servo object


  // Robot home position
  base.write(0);
  delay(15);
  arm1.write(45);
  delay(150);
  arm2.write(90);
  delay(15);
  
  pinMode(16, OUTPUT);

} // End of setup.


// This is the Arduino main loop function.
void loop() {

  //loop_timer = micros();
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  while (doConnect == true) {
    if (connectToServer(*pServerAddress)) {
      Serial.println("We are now connected to the BLE Server.");
      digitalWrite(16, HIGH);   // turn the LED on to signify start of calibration (HIGH is the voltage level)
      connected = true;
      const uint8_t sampleOn[] = {0x53, 0x35, 0x02, 0x01, 0x32};
      for (int i = 0; i < 10; i++) {
        pRemoteCharacteristicS->writeValue((uint8_t*)sampleOn, 5, true);
        //delay(100);
      }
      doConnect = false;

      Serial.println("CALIBRATING..........");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
  }



  
    //accelerometer calc 180/3.142=57.296 convert to degress
     angle_pitch_accz=-atan2(-accx,(sqrt((accy*accy)+(accz*accz))))*57.296;//pitch//calc orientation
     angle_roll_accx=-atan2(accz,accy)*57.296;//roll//calc orientation


  if (set_init_angles) { // Implement complimentary filter                             
    f = 0.16;                 // filter cutoff frequency
    to = 0.02;                // sampling time
    rc = (double)1 / (2 * M_PI * f); //  filter time constant  
    alpha = (double)rc / (rc + to); // NB alpha ~= 0.98
    angle_pitchz = ((angle_pitchz + (angle_pitch_gyro * to)) * (alpha) + angle_pitch_accz * (1- alpha)); //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_rollx = ((angle_rollx + (angle_roll_gyro * to)) * (alpha) + angle_roll_accx * (1-alpha));

  } else {                                                               
    angle_pitchz = angle_pitch_accz;                                     //Set the pitch angle equal to the accelerometer pitch angle
    angle_rollx = angle_roll_accx;                                      //Set the  roll angle equal to the accelerometer roll angle
    set_init_angles = true;                                           
  }

  if (isCalibrated == 1) {

    pitch = angle_pitchz - (angle_pitchzOffset);
    roll = angle_rollx - (angle_rollxOffset);

    //x=(x - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;  // Mapping the values to limit range of angles
    pitch = (int)((pitch - 0.0) * (0.0 - 90) ) / (abs(60-angle_pitchzOffset) - 0.0) + 90.0;
    roll = (int)((roll - 0.0) * (90.0 - 0.0) ) / ((45-angle_rollxOffset)- 0.0) + 0.0; 

    // To serial plotter
    Serial.print(pitch);  
    Serial.print(" ");
    Serial.println(roll );


    //Code block to send  signal to  robot arm
    if(angle_pitchz>0&&angle_rollx>0&&pitch>=0&&pitch<=60){//Pitch
      
       arm1.write((int)pitch);
     // Serial.print("PITCHING:");
      // Serial.println((double)pitch);
    }
    if(angle_pitchz<0&&angle_rollx>0&&roll>=0&&roll<=90){//roll
     
      roll = (int)((roll - 20.0) * (90.0 - 0.0) ) / (90- 20.0) + 0.0; 
      base.write((int)roll);
    // Serial.println((double)roll); 
    }else{
     // Serial.println(0);
    }
    // End of sending signal to robot arm
    
  } else if ( (set_init_angles == true)&& isCalibrated == 0) {  // Code block to get offeset for calibration
    Serial.print(angle_rollx);
    Serial.print(" ");
    Serial.println(angle_pitchz);
    angle_rollxOffset = angle_rollxOffset + angle_rollx;
    angle_pitchzOffset = angle_pitchzOffset +  angle_pitchz;

    if (counter == 250) { // get 250 samples when esense is worn and average them
      angle_rollxOffset = (double)angle_rollxOffset / counter;
      angle_pitchzOffset = (double)angle_pitchzOffset / counter;
      Serial.print("OffsetPR:");
      Serial.print((angle_pitchzOffset));
      Serial.print(" ");
      Serial.println((angle_rollxOffset));
     
      Serial.println("Calibrated");

      // Blink LED to signify end of calibration
      digitalWrite(16, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
      digitalWrite(16, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);                       // wait for a second
      digitalWrite(16, LOW);    // turn the LED off by making the voltage LOW
      delay(1000);
      counter = 1;
      isCalibrated = 1;
    }
    counter++;

  }



} // End of loop
