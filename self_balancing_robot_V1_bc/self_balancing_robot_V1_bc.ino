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

float yaw = 0;                           //variable to catch yaw data in degress from IMU
float pitch = 0;                         //variable to catch pitch data in degress from IMU 

/*--------------------------------------------------------------------------------------------------------------------------*/

/*BLUETOOTH VARIABLES AND CONFIGURATION-------------------------------------------------------------------------------------*/
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;

String angle = "";                       //joystick angle when preseed 0-359 degress
String strength = "";                    //joystick strength when pressed 0-100
String button = "";                      //pressed buttons (1-4)
String value = "";                       //total string send from bluetooth app to esp32

float level = 0.0;                       //to change setpoint aggressively.
char move_selector;                      //to select between different robot moves 
/*--------------------------------------------------------------------------------------------------------------------------*/


/*INTERNAL ESP32 LED VARIABLES AND CONFIGURATION----------------------------------------------------------------------------*/
int cont = 0;                            //cont for defined blinks
unsigned long currentTime=0;             //current time from esp32 in millis
unsigned long previousTime=0;            //previous time from esp32 in millis
bool ledState=LOW;                       //initial state of internal LED

/*--------------------------------------------------------------------------------------------------------------------------*/


/*PID VARIABLES AND CONFIGURATION-------------------------------------------------------------------------------------------*/
#include "PID_v1.h"
double kp =18.67;
double ki =243.67;
double kd =0.20;
double setpoint, input, output;         
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

/*--------------------------------------------------------------------------------------------------------------------------*/


/*VOID SETUP CONFIGURATION--------------------------------------------------------------------------------------------------*/
void setup() {
  Serial.begin(115200);
  Setup_motors();
  Setup_MPU6050();
  SerialBT.begin("BALANCING ROBOT");     //Bluetooth device name
  Setup_PID();
}
/*--------------------------------------------------------------------------------------------------------------------------*/


/*LOOP PROGRAM--------------------------------------------------------------------------------------------------------------*/
void loop() {
  bluetooth_app();
  /*
  Serial.print("Angle:"); Serial.print(angle.toInt()); Serial.print('\t');
  Serial.print("Strength:"); Serial.print(strength.toInt()); Serial.print('\t');
  Serial.print("Button:"); Serial.print(button.toInt()); Serial.println('\t');
  */
   
  //Blue potentiometer
  potB_value = float(map(analogRead(potB),0,4095,-300,300))/100; 

  //read IMU data
  Update_MPU6050_DMP();
  pitch = ypr[1] * 180/M_PI;
  input = pitch;
  
  // PID controller sample time
  myPID.SetSampleTime(5);
  
  //case to select between different robot moves
  if (angle.toInt()==0){move_selector='p';} //in place 
  else if (angle.toInt()<45 || angle.toInt() >315) {move_selector = 'l';} //to the left
  else if (angle.toInt()>=45 && angle.toInt() <=135) {move_selector = 'f';} //forward
  else if (angle.toInt()>135 && angle.toInt() <=225) {move_selector = 'r';} //to the right
  else if (angle.toInt()>225 && angle.toInt() <=315) {move_selector = 'b';} //backward
  
  switch(move_selector){
    /*****************************************************/
    case 'p':
    //balance the robot in place
    setpoint = potB_value;
    myPID.Compute();
    speed_motors(output);
    //detects when the robot falls down
    if (pitch >30 || pitch <-30 || pitch == setpoint){
      speed_motors(0);
    }
    break;
    /*****************************************************/
    
    /*****************************************************/
    case 'l':
    //move the robot to the left
    setpoint = potB_value;
    myPID.Compute();
    speed_right_motor(output);
    speed_left_motor(output+20);
    //detects when the robot falls down
    if (pitch >30 || pitch <-30 || pitch == setpoint){
      speed_motors(0);
    }
    break;
    /*****************************************************/
    
    /*****************************************************/
    case 'f':
    //move the robot forward
    if (button.toInt()==3){
      //to change robot tilt aggressively with bluetotth app
      level = 1.5;
    }
    setpoint = potB_value+1+level;
    myPID.Compute();
    speed_motors(output);
    //detects when the robot falls down
    if (pitch >30 || pitch <-30 || pitch == setpoint){
      speed_motors(0);
    }
    break;
    /*****************************************************/
    
    /*****************************************************/
    case 'r':
    //move the robot to the right
    setpoint = potB_value;
    myPID.Compute();
    speed_right_motor(output+20);
    speed_left_motor(output);
    //detects when the robot falls down
    if (pitch >30 || pitch <-30 || pitch == setpoint){
      speed_motors(0);
    }
    break;
    /*****************************************************/
    
    /*****************************************************/
    case 'b':
    //move the robot backward
    if (button.toInt()==1){
      //to change robot tilt aggressively with bluetotth app
      level = 1.5;
    }
    setpoint = potB_value-1-level;
    myPID.Compute();
    speed_motors(output);
    //detects when the robot falls down
    if (pitch >30 || pitch <-30 || pitch == setpoint){
      speed_motors(0);
    }
    break;
    /*****************************************************/
  }
  //to restore robot tilt with bluetotth app
  level = 0;
  /*
  Serial.print("Setpoint: ");
  Serial.println(setpoint);
  */
  
}
/*--------------------------------------------------------------------------------------------------------------------------*/
