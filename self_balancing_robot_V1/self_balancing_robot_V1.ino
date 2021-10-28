/*MOTOR DRIVER VARIABLES AND CONFIGURATION--------------------------------------------------------------------------------*/
// 27 - Standby pin
// 14 - AIN1 pin
// 12 - AIN2 pin
// 13 - PWMA pin
// 26 - BIN1 pin
// 25 - BIN2 pin
// 33 - PWMB pin
// To reverse forward motor direction, switch the AIN1 and AIN2 or BIN1 and BIN2 pin numbers.
const int pwm_left_motor = 13;     
const int pwm_right_motor = 33;
const int standby = 27;
const int motor_left_AIN1 = 14;
const int motor_left_AIN2 = 12;
const int motor_right_BIN1 = 26;
const int motor_right_BIN2 = 25;

const int pwm_frecuency = 980;           // PWM frecuency for arduino uno 
const int pwm_left_channel = 0;          // 0-15 available channels (left motor channel) 
const int pwm_right_channel = 1;         // 0-15 available channels (right motor channel)
const int resolution = 8;                // 8-bit resolution means control values from 0 to 255

/*--------------------------------------------------------------------------------------------------------------------------*/

/*ANALOG INPUT VARIABLES AND CONFIGURATION(POTENTIOMETERS)------------------------------------------------------------------*/
//ADC2 (GPIOS 2,4,12-15,25-27) cannot be used when Wi-Fi is used. GPIOS 34-35 belong to ADC1 (GPIOS 32-39)
const int potA = 34;                     //Potentiometer 5k ohm (orange head)
const int potB = 35;                     //Potentiometer 5k ohm (blue head)
float potA_value =0;                     //variable for storing the potentiometer A value(0-4095)
float potB_value =0;                     //variable for storing the potentiometer B value(0-4095)

      
/*--------------------------------------------------------------------------------------------------------------------------*/


/*MPU6050 VARIABLES AND CONFIGURATION--------------------------------------------------------------------------------------*/ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  /* Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
  is used in I2Cdev.h*/
  #include "Wire.h"
#endif

MPU6050 mpu;                             //class default I2C address is 0x68. Use MPU6050 mpu(0x69) for ADO high 
#define OUTPUT_READABLE_YAWPITCHROLL     // pitch/roll angles (in degrees) calculated from the quaternions coming from FIFO
#define INTERRUPT_PIN 23                 // use the interrupt pin in MPU6050

// MPU control/status variables
bool dmpReady = false;                   // set true if DMP init was successful
uint8_t mpuIntStatus;                    // holds actual interrupt status byte from MPU
uint8_t devStatus;                       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                  // FIFO storage buffer

//orientation/motion variabless
Quaternion q;
VectorFloat gravity;
float ypr[3];                            // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
                                         // packet structure for InvenSense teapot demo
                                        
volatile bool mpuInterrupt = false;      // Interrupt detection routine. Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float pitch = 0;                         //variable to catch pitch data in degress from IMU 

/*--------------------------------------------------------------------------------------------------------------------------*/


/*INTERNAL ESP32 LED VARIABLES AND CONFIGURATION----------------------------------------------------------------------------*/
int cont = 0;                            //cont for defined blinks
unsigned long currentTime=0;             //current time from esp32 in millis
unsigned long previousTime=0;            //previous time from esp32 in millis
bool ledState=LOW;                       //initial state of internal LED

/*--------------------------------------------------------------------------------------------------------------------------*/


/*PID VARIABLES AND CONFIGURATION-------------------------------------------------------------------------------------------*/
#include "PID_v1.h"
double kp = 18.67;
double ki = 0243.67;
double kd = 0;
double setpoint, input, output;         
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

/*--------------------------------------------------------------------------------------------------------------------------*/


/*VOID SETUP CONFIGURATION--------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Setup_motors();
  Setup_MPU6050();
  Setup_PID();
}
/*--------------------------------------------------------------------------------------------------------------------------*/


/*LOOP PROGRAM--------------------------------------------------------------------------------------------------------------*/
void loop() {
  
  potB_value = float(map(analogRead(potB),0,4095,-300,300))/100;   //Blue potentiometer
  //potA_value = float(map(analogRead(potA),0,4095,0,3000))/10;  //Orange potentiometer
  /*erial.print("kp:");
  Serial.print(potB_value);
  Serial.print("\t");
  Serial.print("ki:");
  Serial.println(potA_value);
  */
  setpoint = potB_value;
  
  Update_MPU6050_DMP();       //read the IMU sensor
  pitch = ypr[1] * 180/M_PI;  //pitch value in degress
  input = pitch;              //PID control input variable
  
  myPID.SetSampleTime(5);           //sampling every 5ms
  /*kp = potB_value;                  //set pot.(blue) value float between 0-50 to kp
  ki = potA_value;                  //set pot.(orange)value float between 0-300 to ki        
  myPID.SetTunings(kp,ki,kd);       //set kp,ki,kd parameters
  */
  myPID.Compute();                  //PID control compute

 
  if (pitch >30 || pitch <-30 || pitch == setpoint){
    speed_motors(0);         //turn off the motors
  }
  else{
    speed_motors(output); //set the output PID control to motors
  }
}

/*--------------------------------------------------------------------------------------------------------------------------*/
