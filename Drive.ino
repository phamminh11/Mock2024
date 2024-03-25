

// Enable / disable debug log
#define DEBUG

// For controlling PCA9685
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_PWMServoDriver.h>
#include <chrono>
// For using PS2 controller
#include <PS2X_lib.h>

#define dbg Serial

/******************************************************************
 * Pins config for the library :
 * - On the motorshield of VIA Makerbot BANHMI, there is a 6-pin
 *   header designed for connecting PS2 remote controller.
 * Header pins and corresponding GPIO pins:
 *   MOSI | MISO | GND | 3.3V | CS | CLK
 *    12     13    GND   3.3V   15   14
 ******************************************************************/

#define PS2_DAT 12 // MISO
#define PS2_CMD 13 // MOSI
#define PS2_SEL 15 // SS
#define PS2_CLK 14 // SLK

/******************************************************************
 * Select mode for PS2 controller:
 *   - pressures = Read analog value from buttons
 *   - rumble    = Turn on / off rumble mode
 ******************************************************************/
#define pressures false
#define rumble false

PS2X ps2x; // Create ps2x instance
MPU6050 mpu(0x69);
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); // Create pwm instance
float deadzone = 100.0f;

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double angle = 0.0;


void setup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);/private/var/folders/7y/ltq9dcf16msd51s89cfnjrmw0000gp/T/.arduinoIDE-unsaved2024222-2280-13c2l4.z0v0n/MPU6050_DMP6/MPU6050_DMP6.ino
  #endif
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      mpuIntStatus = mpu.getIntStatus();
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmpReady = true;
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  }
  int error = -1;
  for (int i = 0; i < 10; i++) // thử kết nối với tay cầm ps2 trong 10 lần
  {
    delay(1000); // đợi 1 giây
    // cài đặt chân và các chế độ: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print(".");
    if (error == 0) break;
  }
  // Init motor controller
  pwm.begin(); // Initialize PCA9685 
  pwm.setOscillatorFrequency(27000000); // Set frequency for PCA9685
  pwm.setPWMFreq(50); // PWM frequency. Should be 50-60 Hz for controlling both DC motor and servo
  Wire.setClock(400000); // Set to max i2c frequency @ 400000
  delay(4000);
}

double GetAngle(){
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      double angle_ = ypr[0] * 180.0/M_PI;
      if(angle_ < 0.0) angle_ += 360.0;
      return angle_;
  }
  return 0.0;
}
void SetMotorSpeed(double vl, double vr){
  if(vl > 0){
    pwm.setPWM(8, 0, 0); pwm.setPWM(9, 0, abs(vl));
  }
  else if(vl < 0){
    pwm.setPWM(8, 0, abs(vl)); pwm.setPWM(9, 0, 0);
  }
  else{
    pwm.setPWM(8, 0, 0); pwm.setPWM(9, 0, 0);
  }
  dbg.print(" ");
  if(vr > deadzone){
    pwm.setPWM(15, 0, abs(vr)); pwm.setPWM(14, 0, 0);
  }
  else if(vr < -deadzone){
    pwm.setPWM(15, 0, 0); pwm.setPWM(14, 0, abs(vr));
  }
  else{
    pwm.setPWM(15, 0, 0); pwm.setPWM(14, 0, 0);
  }
}
// Control mode
int bco = 19;

void SplitArcade(int x, int y){
  float vl = (float)(y + x)/255.0*4095.0, vr = (float)(y - x)/255.0*4095.0;
  vl = max(min(vl, 4095.0f), -4095.0f), vr = max(min(vr, 4095.0f), -4095.0f);
  if(vl < deadzone || vl > -deadzone) vl = 0.0;
  if(vr < deadzone || vr > -deadzone) vr = 0.0;
  SetMotorSpeed(vl, vr);
}
void TankDrive(int yl, int yr){
  float vl = (float)yl/128.0f*4095.0f, vr = (float)yr/128.0f*4095.0f;
  SetMotorSpeed(vl, vr);
}
void DriverControl(){
  //SplitArcade(ps2x.Analog(PSS_RX) - 128, -ps2x.Analog(PSS_LY) + 128);
  TankDrive(-ps2x.Analog(PSS_LY) + 128, -ps2x.Analog(PSS_RY) + 128);
  
  //pwm.setPWM(port, 0, speed);
  //Intake
  if(ps2x.ButtonPressed(PSB_L1))  pwm.setPWM(11, 0, 4095);
  if(ps2x.ButtonReleased(PSB_L1))  pwm.setPWM(11, 0, 0);

  int speed = 4095;
  if(ps2x.Button(PSB_BLUE)) speed = 2000;
  //Storage
  if(ps2x.Button(PSB_R1)){
    pwm.setPWM(13, 0, speed); pwm.setPWM(12, 0, 0);
  }
  else if(ps2x.Button(PSB_R2)){
    pwm.setPWM(13, 0, 0); pwm.setPWM(12, 0, speed);
  }
  else{
    pwm.setPWM(13, 0, 0); pwm.setPWM(12, 0, 0);
  }
}

class PID
{
  double derivative, integral, last_error=-1;
  std::chrono::time_point<std::chrono::system_clock> last_time;
  public:
  double kP, kI, kD;
  void setup(double p, double i, double d){
    kP = p; kI = i; kD = d;
  }
  void Move(double dist, double heading){
    while(1){

    }
  }
  void Rotate(double heading){
    last_error = -1;
    while(1){
      angle = GetAngle();
      double output = calculate(angle, heading);
      if(output >  2500.0) output = 2500.0;
      if(output <  -2500.0) output = -2500.0;
      if(output <  300.0 && output > 0.0) output = 300.0;
      if(output >  -300.0 && output < 0.0) output = -300.0;
      Serial.println(output);
      //output = min(max(1000.0, output), 4095.0);
      SetMotorSpeed(output, -output);
      delay(10);
    }
  }
  double calculate(double state, double reference) {
    int dir = 1;
    if(state > reference) dir *= -1; 
    double error = max(reference, state) - min(reference, state);
    if (error > 180.0) {
      error = 360.0-error;
      dir *= -1;
    }
    if (last_error == -1) {
      last_error = error;
      last_time = std::chrono::system_clock::now();
      return 0;
    }
    std::chrono::time_point<std::chrono::system_clock> current_time = std::chrono::system_clock::now();
    double time_passed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_time).count() / 1000.0;
    derivative = (error-last_error)/time_passed;
    integral += (error-last_error) * time_passed;
    last_error = error;
    last_time = std::chrono::system_clock::now();
    return (kP * error + kD * derivative + kI * integral)*dir;
  }
};

bool canRunAuto = true;
void AutonomousControl(){
  if(!canRunAuto) return;
  if(ps2x.ButtonPressed(PSB_GREEN) ){
    PID pid;
    pid.setup(70.0, 40.0, 0.0);
    pid.Rotate(90.0);
    canRunAuto = false;
  }
}

void loop() {
  ps2x.read_gamepad(false, false);
  //AutonomousControl();
  DriverControl();
  // Read the gamepad state
  delay(1000);
}