#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP085.h>
#include <utility/imumaths.h>
#include <AutoPID.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
File accelFile;
File heightFile;
Servo latchservo;
Servo xservo;
Servo yservo;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_BMP085 bmp;
#define OUTPUT_MIN -1000
#define OUTPUT_MAX 1000
#define KP 1
#define KI 0
#define KD 0
const int rightbuttonpin = 7;
const int leftbuttonpin = 8;
const int LED_PIN    = 9;
//int ledState = LOW;
double state = 0;
int pos = 0;
double inputx, inputy, outputValx, outputValy;
double setPointx,setPointy;

AutoPID PIDX(&inputx, &setPointx, &outputValx, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PIDY(&inputy, &setPointy, &outputValy, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
double nums[]= {0,0,0,0};
double maxheight=0;


void setup() {
  Serial.begin(115200);
  heightFile = SD.open("altitude", FILE_WRITE);
  accelFile = SD.open("acceleration", FILE_WRITE);
  xservo.attach(2);
  xservo.write(90);
  yservo.attach(3);
  yservo.write(90);
  PIDX.setTimeStep(1);
  PIDY.setTimeStep(1);
  pinMode(LED_PIN, OUTPUT);
  pinMode(rightbuttonpin, INPUT);
  pinMode(leftbuttonpin, INPUT);
  latchservo.attach(4);
  latchservo.write(0);
  //SENSOR START UP /////////////////////////
  if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  while (1) {}
  }
  if (!bno.begin())
  {
    
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  /*
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  */
  Serial.println("initialization done.");
/////////////////////////////////////////////////////////////////////////
}

void loop() {
  //DEAD STATE
  if (state == 0) {
    if (digitalRead(leftbuttonpin)==HIGH) {
      state = .5;
    }
    //SERVO BUTTON*************************
    if (digitalRead(rightbuttonpin)==HIGH) {
      pos = latch(pos);
      delay(100);
    }
  }
//CALIBRATION STATE
  if (state == .5) {
    blinky(1000);
    if (calibrate() == true) {
      state = 1;
    }
  }
//WAITING FOR BUTTON PRESS
  if (state == 1) {
    
    blinky(100);
   // digitalWrite(LED_PIN, HIGH);
  
    if (digitalRead(leftbuttonpin)==HIGH) {
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      setPointx = euler.x();
      setPointy = euler.y();
      state = 2;
    }
    if (digitalRead(rightbuttonpin)==HIGH) {
      pos = latch(pos);
      delay(100);
    }
  }

  if (state == 2) {
    digitalWrite(LED_PIN, HIGH);
    if (takeoff() == true) {
      state = 3;
    }
    
    while (state == 3) {
      //data();
      //PIDXF();
      //PIDYF();
      //delay(100);
      //Serial.println("itry");
     detectoff(); 
    }
    
    if (state == 4) {
      //calculate when to go off
      //temp
      Serial.println("ibad");
      pos = latch(pos);
      state=5;

    }
    
  }
  Serial.println(state);
}


//Methods
bool calibrate() {
  //change?
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  if (mag == 3) {
    return true;
  }
  else {
    return false;
  }
}
void blinky(int timer) {
  digitalWrite(LED_PIN, HIGH);
  delay(timer);
  digitalWrite(LED_PIN, LOW);
  delay(timer);
}

int latch(int pos) {
  if (pos == 0) {
    latchservo.write(90);
    pos = 1;
    return pos;
  }
  else {
    latchservo.write(0);
    pos = 0;
    return pos;
  }
}
bool takeoff() {
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  double xa = acceleration.x();
  double ya = acceleration.y();
  double za = acceleration.z();
  float at = sqrt(xa * xa + ya * ya + za * za);     // total acceleration
  Serial.println(at);
  if (abs(at) < 10) {
    return true;
  }
  else {
    return false;
  }
}
void PIDXF() {

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  inputx = (euler.x());
  
    
    PIDX.run();
    //if((setPointx-outputValx)>180){
    xservo.write(90+outputValx);
   Serial.print(setPointx);
    Serial.print("   ");
    Serial.print(euler.x());
    Serial.print("   ");
    Serial.println(90+outputValx);
    /*
    }
    else{
    xservo.write(setPointx-outputValx);
    
    Serial.print(setPointx);
    Serial.print("   ");
    Serial.print(euler.x());
    Serial.print("   ");
    Serial.println(setPointx-outputValx);
    }*/
 

}
void PIDYF() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  inputy = (euler.y());
  if (inputy > 180) {
    inputy = (360 - inputy);
    PIDY.run();
    yservo.write(90 + abs(outputValy));
    Serial.print(euler.y());
    Serial.print("   ");
    Serial.print(outputValy);
  }
  else if (inputy <= 180) {
    PIDY.run();
    yservo.write(90 + outputValy);
    Serial.print("               ");
    Serial.print(euler.y());
    Serial.print("   ");
    Serial.println(outputValy);
  }
}

void detectoff() {
  
  int pos = nums[3];
  nums[pos]=bmp.readAltitude();
  if (nums[3]==2)nums[3]=0;
  else nums[3]=nums[3]++;
  double avg=(nums[0]+nums[1]+nums[2])/3;
  if (avg+1>=maxheight)maxheight=avg;
    else state=4;
    Serial.println(maxheight);
    
    delay(100);
  

}

void data() {
  imu::Vector<3> acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  //change
  accelFile.println(acceleration.x(),acceleration.y());
  accelFile.close();
  heightFile.println(bmp.readAltitude());
  heightFile.close();

}
