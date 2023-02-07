

// including library for I2C communication
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


//  including Servo motor library
#include <Servo.h>

Servo servol;// instance generated for left servo motor
Servo servor;// instance generated for right servo motor

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;// instance created for retrieving yaw angle from 6- axis gyroscope and accelerometer
//MPU6050 mpu(0x69); // <-- use for AD0 high



#define INTERRUPT_PIN 2  //
#define LED_PIN 13       // 
#define LED 10


const int buttonPin = 9;
const int pingPin = 3;    // utrasonic sensor
const int IRSensorL = 4;  // IR Sensor left
const int IRSensorR = 5;  // IR Sensor  right
// MPU control/status vars
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorInt16 aa;       // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;   // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;  // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];       // [psi, theta, phi]    Euler angle container
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// 
//              INTERRUPT DETECTION ROUTINE                ===


volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

// defining variables for home position
float x, y = 0;  //position


float dxs;  // = 120;  //desired state for service station
float dys;  //= 120;
float dxt;  // = 120;  //desired state for table number
float dyt;  //= 120;

 // variable to track delta time
float state_update_lasttime = 0;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
    Serial.begin(9600);
    pinMode(IRSensorL, INPUT);  // sensor pin INPUT
    pinMode(IRSensorR, INPUT);  // sensor pin INPUT
    while (!Serial);  
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();

    // gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(68);      //(220);
    mpu.setYGyroOffset(-15);     //(76);
    mpu.setZGyroOffset(-16);     //-85
    mpu.setXAccelOffset(-3834);  //(-4269);
    mpu.setYAccelOffset(318);    //(-4836);
    mpu.setZAccelOffset(2218);   //(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      // ////Serial.pr(F("DMP Initialization failed (code "));
      //////Serial.pr(devStatus);
      //  ////Serial.prln(F(")"));
    }

    /// servo motor
    servol.attach(7);
    servor.attach(8);

    // configure LED for output and push button for having input
    pinMode(LED, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(buttonPin, INPUT);
}

// constant speed hard coded after doing hit and trial
float speed = 13;  //7.6;  //cm/second
float time;
int yini = 0;
float actual_yaw;
//                  MAIN PROGRAM LOOP                    

void loop() {
  
  MotorBrake();// to ensure left motor and right motor does not move
  String readString;
  // initial state setting to 0,0
  x=0;y=0;
  // not moving forward untill the message is not received from the esp32
  while (Serial.available() == 0) {
    //Check Serial Port
  }
  // reading the message from ESP32
  while (Serial.available()) {
  
    if (Serial.available() > 0) {
      readString = Serial.read();  //gets one byte from serial buffer

     
    }
  }
  
  if (yini == 1) {
    Desire_state(0, 100);// go to service station
    yini = 2;
    // code to check for button press
    while (digitalRead(buttonPin) == 0) {
      delay(1);
    }
    Desire_state(100, 40);// go to table
    // check for button press 
    while (digitalRead(buttonPin) == 0) {
      delay(1);
    }
    Desire_state(0, 0);// go to home 
    Rotate(0);// straight orient in home 
  }

  // First fuction to desired state
  if (yini == 0) {

    Desire_state(0, 60);
    yini = 1;
    // code to check for button press
    while (digitalRead(buttonPin) == 0) {
      delay(1);
    }

    Desire_state(100, 90);

    
    // check for button press check for
    while (digitalRead(buttonPin) == 0) {
      delay(1);
    }
   
  
    Desire_state(0, 0);
    delay(2);

    Rotate(0);// setting back to initial orientation
  }
}

void Desire_state(float xd, float yd) {
RE:
  float desired_yaw = atan2(abs(yd - y), abs(xd - x)); //computing desired yaw ,x and y are current states
  // change desired_yaw according to imu frame
  if (yd - y >= 0 && xd - x >= 0) {
    desired_yaw = M_PI / 2 - desired_yaw;// 1st quadrant
  } else if (yd - y < 0 && xd - x > 0) {
    desired_yaw = M_PI / 2 + desired_yaw;// 4rd quadrant
  } else if (yd - y <= 0 && xd - x <= 0) {
    desired_yaw = -(M_PI / 2 + desired_yaw);// 3rd quadrant
  } else {
    desired_yaw = -desired_yaw; // 2nd quadrant
  }
  desired_yaw = desired_yaw * (180 / M_PI);  // in degrees
  //Now first rotating the robot in desired yaw direction
  Rotate(desired_yaw);
  float distance = sqrt((xd - x) * (xd - x) + (yd - y) * (yd - y));// // distance and time calculate in cm
  time = (distance / speed);// in seconds
  int d = Straight(time, desired_yaw); // function to move in straight line
  if (d == 0) {
    /* this means desired state has not been reached, obstacle came in between run obstacle avoidance
     routine untill a state arrives where feedback from all the three sensors infers no obstacles around*/
    obstacle_avoidance_routine();
    // at this point obstacle has been avoided, but destination has not yet been reached.
    // therefore going back to start of this function.
    goto RE;
  }
}


int Straight(float td, float dyaw) {
  // this function is used to move in staright line
  state_update_lasttime = millis() / 1000;  // in seconds, to note the initial time
  //PID gain constants
  int Kp = 1;
  int Ki = 0.02;
  int Kd = .9;
  // error trackers varibles
  float error = 0;
  float errSum = 0;
  float lastErr = 0;
  float current_millis = millis() / 1000;
  float checker = (millis() / 1000) - current_millis;
  while (checker <= td) {
    int checkObstacle = obstacle_present();
    if (checkObstacle == 2 || checkObstacle == 3 || checkObstacle == 6) {
      MotorBrake();// stops both the motor
      State_Update_1((float)(millis() / 1000), dyaw);// updates the state(x,y)
      return 0;
    }
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      actual_yaw = ypr[0] * (180 / M_PI);
      error = actual_yaw - dyaw;// error used for PID
      errSum = errSum + error;
      if (error < -1.5 || error > 1.5) {
        servol.attach(7);
        servol.write(90 - (Kp * (error) + Ki * errSum + Kd * (error - lastErr)));  // backward  90 is stop
        servor.attach(8);
        servor.write(90 - (Kp * (error) + Ki * errSum + Kd * (error - lastErr)));  // forward   90 is stop
      } else {
        Forward_Drive();// this functions make the two motors run at a constant speed
      }
      lastErr = error;
    }
    checker = (millis() / 1000) - current_millis;
  }
  MotorBrake();
  State_Update_1((float)(millis() / 1000), dyaw);// updates the state(x,y)
  return 1;
}



void Rotate(float desired_yaw) {
  int Kp = 1.9; //PID gain constants
  int Ki = 0.02;
  int Kd = .5;
  float error = 0;// error trackers varibles
  float errSum = 0;
  float lastErr = 0;
  while (1) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      actual_yaw = ypr[0] * (180 / M_PI);
      error = actual_yaw - desired_yaw;
      errSum = errSum + error;
      if (error < -1 || error > 1) {
        servol.attach(7);
        servol.write(90 - (Kp * (error) + Ki * errSum + Kd * (error - lastErr)));  //   90 is stop
        servor.attach(8);
        servor.write(90 - (Kp * (error) + Ki * errSum + Kd * (error - lastErr)));  //   90 is stop
      } else {
        MotorBrake();
        break;
      }
      lastErr = error;
    }
  }
}

void obstacle_avoidance_routine() {
  float aux1;int aux2;
  int ostate = obstacle_present();
  //000  : 0 no obstacle, leave  obs //001,  : 1 straight //010, : 2 left turn //011  :  3 left
  //100  : 4 straight //101  : 5 straight //110  : 6  right turn //111  : wont arrive in this situation
  while (ostate != 0) {
    ostate = obstacle_present();
    switch (ostate) {
      case 1:
        aux2 = Straight(2, aux1);break;
      case 2:
        MotorBrake();// stops both the motor
        aux1 = Left_Drive(60);aux2 = Straight(2, aux1);break;
      case 3:
        MotorBrake();
        aux1 = Left_Drive(60);aux2 = Straight(2, aux1);break;
      case 4:
        aux2 = Straight(2, aux1);break;
      case 5:
        aux2 = Straight(2, aux1);break;
      case 6:
        MotorBrake();
        aux1 = Right_Drive(60);aux2 = Straight(2, aux1);break;
      default:  // for 0
        MotorBrake();break;
    }
  }
}

float Right_Drive(float angle) {
  // rotate by any desired  degrees  anticlockwise
  float current_yaw;
  int aux;i = 1;
  while (i) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      current_yaw = ypr[0] * (180 / M_PI);  // in degrees
      i = 0;
      aux = current_yaw + angle;//converting angle in imu frame
      if (aux > 179) {
        aux = -(360 - current_yaw - angle);// converting angle in imu frame
      }
      Rotate(aux);
    }
  }
  return aux;
}

float Left_Drive(float angle) {
  // rotate by any degrees  anticlockwise
  float current_yaw;
  float aux;
  int i = 1;
  while (i) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      current_yaw = ypr[0] * (180 / M_PI);  // in degrees
      i = 0;
      aux = current_yaw - angle;  // in imu frame
      if (aux < -179.9) {
        aux = 360 + current_yaw - angle;// in imu frame 
      }
      Rotate(aux);
    }
  }
  return aux;
}

void Forward_Drive() {
  servol.attach(7);
  servol.write(180);  // forward  90 is stop

  servor.attach(8);
  servor.write(0);  // forward   90 is stop
}

void MotorBrake() {
  servol.attach(7);
  servol.write(90);  //  90 is stop

  servor.attach(8);
  servor.write(90);  //  90 is stop
}



void State_Update(float time) {
  //  timed = timed / 1000;
  float yaw;
  float timed = time - state_update_lasttime;
  state_update_lasttime = time;  // in seconds
  int i = 1;
  while (i) {

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {  // Get the Latest packet
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      yaw = ypr[0] * (180 / M_PI);  // in degrees
      if (yaw >= 0) {
        yaw = 90 - yaw;
        // yaw will be taken into  a variable
        ////Serial.pr(" Actual Yaw : inside update");
        ////Serial.pr(yaw);
        ////Serial.pr(" \t\n");
        x = x + ((speed * abs(timed)) * cos(yaw));
        y = y + ((speed * abs(timed)) * sin(yaw));
        ////Serial.pr(" position : x  ");
        ////Serial.pr(x);
        ////Serial.pr(" position : y  ");
        ////Serial.pr(y);
        ////Serial.pr(" \t\n");
      } else {
        yaw = (-1 * yaw);
        x = x - ((speed * timed) * sin(yaw));
        y = y + ((speed * timed) * cos(yaw));
      }
      i = 0;
    }
  }
}

int obstacle_present() {
  //000  :  0 no obstacle, leave  obs //001, : 1 straight
  //010, :  2 left turn              //011  :  3 left
  //100  :  4 straight               //101  : 5 straight
  //110  :  6  right turn           //111  : wont arrive in this situation
  int resultU = ultrasonic_value();// in cm
  int resultIR = IR_Module();// digital 
  int mid = 0; left = 0;right =0;
  if (resultU < 15) {
    mid = 1;
  }
  if (resultIR == 0) {
    left = 0;
    right = 0;
  } else if (resultIR == 1) {
    left = 1;
    right = 0;
  } else if (resultIR == 2) {
    left = 0;
    right = 1;
  } else {
    left = 1;
    right = 1;
  }

  return (4 * left + 2 * mid + 1 * right);
}

int ultrasonic_value() {
  // establish variables for duration of the ping, and the distance result
  // in inches and centimeters:
  long duration, inches, cm;

  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  // The same pin is used to read the signal from the PING))): a HIGH pulse
  // whose duration is the time (in microseconds) from the sending of the ping
  // to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  // convert the time into a distance
  cm = microsecondsToCentimeters(duration);
  return cm;
}


int IR_Module() {
  //function output
  // if left sensor senses, output : 1
  // if right sensor senses, output : 2
  // if both sensor senses, output : 3
  // if none of the sensor senses, output : 0 
  int statusSensorL = digitalRead(IRSensorL);
  int statusSensorR = digitalRead(IRSensorR);

  if (statusSensorL == 1 && statusSensorL == 1) {
    return 3;
  } else if (statusSensorL == 1 && statusSensorL == 0) {
    return 1;
  } else if (statusSensorL == 0 && statusSensorL == 1) {
    return 2;
  } else {
    return 0;
  }
}

long microsecondsToInches(long microseconds) {
  // According to Parallax's datasheet for the PING))), there are 73.746
  // microseconds per inch (i.e. sound travels at 1130 feet per second).
  // This gives the distance travelled by the ping, outbound and return,
  // so we divide by 2 to get the distance of the obstacle.
  // See: https://www.parallax.com/package/ping-ultrasonic-distance-sensor-downloads/
  return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the object we
  // take half of the distance travelled.
  return microseconds / 29 / 2;
}

void State_Update_1(float time, float yaw) {
  float timed = time - state_update_lasttime;
  // here yaw comes in imu frame
  // it needs to be transferred to world frame.
  yaw = 90 - yaw;
  yaw = yaw * (M_PI / 180);
  x = x + ((speed * timed) * cos(yaw));
  y = y + ((speed * timed) * sin(yaw));
}
