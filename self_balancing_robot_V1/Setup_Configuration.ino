
/*SETUP MOTORS CONFIGURATION------------------------------------------------------------------------------------------------*/
void Setup_motors(){
  pinMode(pwm_left_motor,OUTPUT);                                // declare GPIOS as output for write values
  pinMode(pwm_right_motor,OUTPUT);
  pinMode(standby,OUTPUT);
  pinMode(motor_left_AIN1,OUTPUT);
  pinMode(motor_left_AIN2,OUTPUT);
  pinMode(motor_right_BIN1,OUTPUT);
  pinMode(motor_right_BIN2,OUTPUT);
  
  digitalWrite(standby,HIGH);                                     //turn on motor driver
  ledcSetup(pwm_left_channel, pwm_frecuency, resolution);         //configure pwm funtionalities for left motor  
  ledcSetup(pwm_right_channel, pwm_frecuency, resolution);        //configure pwm funtionalities for right motor
  ledcAttachPin(pwm_left_motor, pwm_left_channel);                //attach the channel to the gpio used for pwm left motor 
  ledcAttachPin(pwm_right_motor, pwm_right_channel);              //attach the channel to the gpio used for pwm right motor 
}
/*---------------------------------------------------------------------------------------------------------------------------*/



/*SETUP MPU6050 CONFIGURATION------------------------------------------------------------------------------------------------*/
void Setup_MPU6050()
{
  pinMode(LED_BUILTIN, OUTPUT);                            //esp32 internal led
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);                    //esp32 GPIO as external interrupt
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE         // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();
    
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif

  
  mpu.initialize();
  //delay(100);
  if(mpu.testConnection() == true){
      blink_led(6,200);                                    //blink led 3 times(3 ON + 3 OFF = 6) every 200ms
  }
  devStatus = mpu.dmpInitialize();
 
  mpu.setXGyroOffset(5);                                 // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setYGyroOffset(-27);
  mpu.setZGyroOffset(7);
  mpu.setXAccelOffset(455);
  mpu.setYAccelOffset(271);
  mpu.setZAccelOffset(1210); 

  if(devStatus == 0){
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6); 
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }else{;} 
   
}
/*---------------------------------------------------------------------------------------------------------------------------*/

/*SETUP PID------------------------------------------------------------------------------------------------------------------*/
void Setup_PID(){
  myPID.SetOutputLimits(-175, 175);
  myPID.SetMode(AUTOMATIC);
  
}
/*---------------------------------------------------------------------------------------------------------------------------*/
