// for the first 100 -> 490
// for the second 100 -> 490
// for the third 100 -> 470
// for the fourth 100 -> 470
// for the fivth 100 -> 470

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVOMIN  95 // This is the 'minimum' PWM length count (out of 4096)
#define SERVOMAX  450 // This is the 'maximum' PWM length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// int degree, servo = 0;

void moveServo(int servoNum, int targetPos, int currentPWM)
{
  int targetPWM = map(targetPos, 0, 180, SERVOMIN, SERVOMAX);

//   Serial.println(String(targetPos) + " , " + String(targetPWM) + " , " + String(currentPWM));

//  if (targetPWM > currentPWM)
//  {
//    for (int PWM = currentPWM; PWM < targetPWM; PWM++)
//    {
      pwm.setPWM(servoNum, 0, targetPWM);
//      delay(5);
//    }
//  }
//  else if (targetPWM < currentPWM)
//  {
//    for (int PWM = currentPWM; PWM > targetPos; PWM--)
//    {
      pwm.setPWM(servoNum, 0, targetPWM);
//      delay(5);
//    }
//  }
//  else
//    return;

//   Serial.println(String(servoNum) + " Done.");
}

void readmsg()
{
  int numValues = 5;
  String values[numValues];
  int comma_position = 0;
  int start = 0;
  String msg = Serial.readStringUntil('\n');
  if (msg.startsWith("$")) 
  { 
    msg.trim(); 
    msg.remove(0, 1); 

    // Split the string into an array of substrings
    for (int i = 0; i < numValues; i++)
    {
      comma_position = msg.indexOf(',', start);
      if (comma_position == -1)
      {
        // This is the last value, so we can just take the rest of the string
        values[i] = msg.substring(start);
      }
      else
      {
        values[i] = msg.substring(start, comma_position);
        start = comma_position + 1;
      }
    }
  }

  int rotation_deg ;
  for (int i = 0 ; i < 5 ; i++) 
  {
    rotation_deg = values[i].toInt();
    if(i == 1 || i == 2)
      rotation_deg = map(rotation_deg,0,180,180,0);
    moveServo(i, rotation_deg, pwm.getPWM(i, true));
  }

}

void sendReply()
{
    int v[5];
    for(int i = 0; i < 5; i++)
    {
      if(pwm.getPWM(i, true) == 95 || pwm.getPWM(i, true) == 450)
        v[i] = map(pwm.getPWM(i, true),95,450,0,180);
      else
        v[i] = map(pwm.getPWM(i, true),95,450,0,180)+1;
    }
    String backmsg = "$" + String(v[0]) + "," + String(v[1]) + "," + String(v[2]) + "," + String(v[3]) + "," + String(v[4]) + "\n";
    Serial.print(backmsg);
}

void setup() {
  Serial.begin(57600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for (int i = 0; i < 5; i++)
  {
    moveServo(i, 90, pwm.getPWM(i, true));
  }
  delay(1000);
}

void loop()
{
  if (Serial.available())
  {
    readmsg();
    sendReply();
  }
  delay(20);
}
