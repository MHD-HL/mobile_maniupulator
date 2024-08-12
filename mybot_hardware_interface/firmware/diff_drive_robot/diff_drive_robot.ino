#include <AccelStepper.h>

const int left_step_pin = 9;
const int left_dir_pin = 10;

const int right_step_pin = 7;
const int right_dir_pin = 8;

AccelStepper left_motor(AccelStepper::DRIVER, left_step_pin, left_dir_pin);
AccelStepper right_motor(AccelStepper::DRIVER, right_step_pin, right_dir_pin);

int comma_position;
String msg,left_motor_speed, right_motor_speed;

void setup() 
{
  Serial.begin(57600);
  
  left_motor.setMaxSpeed(1000.0);
  right_motor.setMaxSpeed(1000.0);

  left_motor.setAcceleration(500.0);
  right_motor.setAcceleration(500.0);
}

void loop() 
{
  if (Serial.available())
  {
    parseCommand();
    sendReply();
  }
  
  left_motor.runSpeed();
  right_motor.runSpeed(); 

  delay(10);  
}

void parseCommand()
{
    msg = Serial.readStringUntil('\n');
    if (msg.startsWith("$")) 
    { 
      msg.trim(); 
      msg.remove(0, 1); 

      int comma_position = msg.indexOf(','); 
      if (comma_position != -1) 
      {
        left_motor_speed = msg.substring(0, comma_position);
        right_motor_speed = msg.substring(comma_position + 1);

        left_motor.setSpeed(left_motor_speed.toInt()); 
        right_motor.setSpeed(-right_motor_speed.toInt());
      }
    }
}

void sendReply()
{
  Serial.print("$");
  Serial.print(left_motor.currentPosition());
  Serial.print(",");
  Serial.print(right_motor.currentPosition());
  Serial.print(",");
  Serial.print(left_motor.speed());
  Serial.print(",");
  Serial.println(right_motor.speed());
}
