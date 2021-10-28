/*MOTORS FUNCTIONS---------------------------------------------------------------------------------------------------------*/
void speed_right_motor(int right)                         
{
  if(right >= 0)                                    
  {
    digitalWrite(motor_right_BIN1,LOW);                  
    digitalWrite(motor_right_BIN2,HIGH);
    ledcWrite(pwm_right_channel, right);
  }
  
  if(right < 0)                                     
  {                            
    digitalWrite(motor_right_BIN1,HIGH);                  
    digitalWrite(motor_right_BIN2,LOW);
    ledcWrite(pwm_right_channel, abs(right));
  }
}


void speed_left_motor(int left)                         
{
  if(left >= 0)                                    
  {
    digitalWrite(motor_left_AIN1,LOW);                  
    digitalWrite(motor_left_AIN2,HIGH);
    ledcWrite(pwm_left_channel, left);
  }
  
  if(left < 0)                                     
  {                            
    digitalWrite(motor_left_AIN1,HIGH);                  
    digitalWrite(motor_left_AIN2,LOW);
    ledcWrite(pwm_left_channel, abs(left));
  }
}


void speed_motors(int robot_speed)                         
{
  if(robot_speed >= 0)                                    
  {
    digitalWrite(motor_left_AIN1,LOW);                  
    digitalWrite(motor_left_AIN2,HIGH);
    digitalWrite(motor_right_BIN1,LOW);                  
    digitalWrite(motor_right_BIN2,HIGH);
    ledcWrite(pwm_right_channel, robot_speed);
    ledcWrite(pwm_left_channel, robot_speed);
  }
  
  if(robot_speed < 0)                                     
  {                            
    digitalWrite(motor_left_AIN1,HIGH);                  
    digitalWrite(motor_left_AIN2,LOW);
    digitalWrite(motor_right_BIN1,HIGH);                  
    digitalWrite(motor_right_BIN2,LOW);
    ledcWrite(pwm_right_channel, abs(robot_speed));
    ledcWrite(pwm_left_channel, abs(robot_speed));
  }
}
/*---------------------------------------------------------------------------------------------------------------------------*/


/*MPU6050 FUNCTIONS----------------------------------------------------------------------------------------------------------*/

void Update_MPU6050_DMP(){
  if (!dmpReady) return;                                  // if programming failed, don't try to do anything
  while (!mpuInterrupt && fifoCount < packetSize) {       // wait for MPU interrupt or extra packet(s) available
    if (mpuInterrupt && fifoCount < packetSize) { 
      fifoCount = mpu.getFIFOCount();                     // try to get out of the infinite loop
    }  
  }
  
  mpuInterrupt = false;                                   // reset interrupt flag and get INT_STATUS byte
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();                         //get current FIFO count
    
  if ((mpuIntStatus & 0x10) || fifoCount > 512) {
    mpu.resetFIFO();                                      // reset so we can continue cleanly
  }
  else if (mpuIntStatus & 0x02) {                         // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);           // read a packet from FIFO
      fifoCount -= packetSize;                            // (this lets us immediately read more without waiting for an interrupt)
      
      #ifdef OUTPUT_READABLE_YAWPITCHROLL                // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        /*
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        */
      #endif
   }    
}
/*---------------------------------------------------------------------------------------------------------------------------*/


/*BLINK INTERNAL LED FUNCTION------------------------------------------------------------------------------------------------*/
void blink_led(int n_blinks, int time_ms){
  while (cont <= n_blinks){
    currentTime=millis();
    if((currentTime-previousTime)>time_ms){
      previousTime=currentTime;
      ledState=!ledState;
      digitalWrite(LED_BUILTIN,!ledState);
      cont+=1;
    }
  }
}
/*---------------------------------------------------------------------------------------------------------------------------*/
