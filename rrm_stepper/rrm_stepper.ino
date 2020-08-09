#include "Timer.h"

#define PIN_LED 14
#define DEF_STEP_TIME 300
#define STEP_TIMER_RESOLUTION 1
#define DEF_STEP_DIR left
#define DEF_ACK_TIME 500 //time after which if acknowledge isn't received, motor stops
#define MOTOR_STEP 9 //degrees * 10
#define HALF_STEP_POWER 255
#define QUARTER_STEP_POWER 130
#define SERIAL_BAUD 9600
#define SERIAL_TIMOUT 5


/*************************************
 * Treba si pamatat v akom kroku skoncil motor,
 * aby sme nestracali krok pri zmene smeru a
 * zastaveni a opetovnom spusteni motora.
 * Ako otestovat, ze sme nejaky krok stratili?
 */

enum direction {
  left,
  right
};

//Timer led_time;
Timer step_timer;
Timer watchdog_timer;
bool led_state = false;
direction motor_direction = DEF_STEP_DIR;
int motor_state = 0;
bool motor_enabled = false;
bool watchdog = false;
int test = 0;
int step_time = DEF_STEP_TIME;
int step_time_counter = 0;
bool period_changed = false;
String read_msg_num = "";
int motor_pose = 0;

void setup() {
  //establish motor direction toggle pins
  pinMode(12, OUTPUT); //CH A -- HIGH = forwards and LOW = backwards???
  pinMode(13, OUTPUT); //CH B -- HIGH = forwards and LOW = backwards???
  
  //establish motor brake pins
  pinMode(9, OUTPUT); //brake (disable) CH A
  pinMode(8, OUTPUT); //brake (disable) CH B
  
  pinMode(PIN_LED, OUTPUT);
  //led_time.every(1000, ledToggle);
  step_timer.every(STEP_TIMER_RESOLUTION, stepMotor);
  watchdog_timer.every(DEF_ACK_TIME, watchDog);
  Serial.begin(SERIAL_BAUD);
  Serial.setTimeout(SERIAL_TIMOUT);
}

void loop() {
  //led_time.update();
  step_timer.update();
  watchdog_timer.update();
  if (period_changed)
  {
    step_time = read_msg_num.toInt();
    period_changed = false;
  }
}

void serialEvent()
{
  String read_msg = Serial.readStringUntil(0);
  //Serial.read();
  if (read_msg == "ACK")
  {
    watchdog = true;
  }
  else if (read_msg == "STR")
  {
    motor_enabled = true;
    Serial.println("STR");
  }
  else if (read_msg == "STP")
  {
    motor_enabled = false;
    Serial.println("STP");
  }
  else if (read_msg == "LFT")
  {
    motor_direction = left;
    Serial.println("LFT");
  }
  else if (read_msg == "RGH")
  {
    motor_direction = right;
    Serial.println("RGH");
  }
  else if (read_msg == "PRD")
  {
    Serial.println("PRD");
    Serial.flush(); 
    read_msg_num = Serial.readStringUntil(0);
    period_changed = true;
    //int read_period = read_msg.toInt();
    Serial.flush();
    Serial.println(String(read_msg_num));
    //step_time = read_period;   
  }
}

void ledToggle()
{
  if (led_state == true)
  {
    digitalWrite(PIN_LED, LOW);
    led_state = false;
  }
  else
  {
    digitalWrite(PIN_LED, HIGH);
    led_state = true;
  }
}

void watchDog()
{
  if (watchdog == false)
  {
    motor_enabled = false;
  }
  watchdog = false;
}

void stepMotor()
{
  step_time_counter++;
  if (step_time_counter >= step_time)
  {
    step_time_counter = 0;
    if (motor_enabled == true)
    {
      ledToggle();
      if (motor_direction == left)
      {
        motor_pose += MOTOR_STEP;
        if (motor_pose >= 3600) motor_pose = 0;
        
        switch(motor_state)
        {
          case 0:
            digitalWrite(9, LOW);  //ENABLE CH A
            digitalWrite(8, HIGH); //DISABLE CH B
          
            digitalWrite(12, HIGH);   //Sets direction of CH A
            analogWrite(3, 255);   //Moves CH A
            
            motor_state++;
            break;

          case 1:
            digitalWrite(8, LOW); //ENABLE CH B
          
            digitalWrite(13, LOW);   //Sets direction of CH B
            analogWrite(11, 255);   //Moves CH B
            
            motor_state++;
            break;
          
          case 2:
            digitalWrite(9, HIGH);  //DISABLE CH A
            //digitalWrite(8, LOW); //ENABLE CH B
          
            //digitalWrite(13, LOW);   //Sets direction of CH B
            analogWrite(11, 255);   //Moves CH B
            
            motor_state++;
            break;

          case 3:
            digitalWrite(9, LOW);  //ENABLE CH A

            digitalWrite(12, LOW);   //Sets direction of CH A
            analogWrite(3, 255);   //Moves CH A
            motor_state++;
            break;
            
          case 4:
            //digitalWrite(9, LOW);  //ENABLE CH A
            digitalWrite(8, HIGH); //DISABLE CH B
          
            //digitalWrite(12, LOW);   //Sets direction of CH A
            analogWrite(3, 255);   //Moves CH A
      
            motor_state++;
            break;

          case 5:
            digitalWrite(8, LOW); //ENABLE CH B
          
            digitalWrite(13, HIGH);   //Sets direction of CH B
            analogWrite(11, 255);   //Moves CH B
            motor_state++;
            break;
          
          case 6:
            digitalWrite(9, HIGH);  //DISABLE CH A
            //digitalWrite(8, LOW); //ENABLE CH B
          
            //digitalWrite(13, HIGH);   //Sets direction of CH B
            analogWrite(11, 255);   //Moves CH B
      
            motor_state++;
            break;

          case 7:
            digitalWrite(9, LOW);  //ENABLE CH A

            digitalWrite(12, HIGH);   //Sets direction of CH A
            analogWrite(3, 255);   //Moves CH A
            motor_state = 0;
            break;
          
        }
      }
      else
      {
        motor_pose -= MOTOR_STEP;
        if (motor_pose < 0) motor_pose = 3600 - MOTOR_STEP;

        switch(motor_state)
        {
          case 0:
            //digitalWrite(9, LOW);  //ENABLE CH A  //////////
            digitalWrite(8, HIGH); //DISABLE CH B
          
            //digitalWrite(12, HIGH);   //Sets direction of CH A ///////
            analogWrite(3, 255);   //Moves CH A
            
            motor_state++;
            break;
          case 1:
            digitalWrite(8, LOW); //ENABLE CH B
            
            digitalWrite(13, HIGH);   //Sets direction of CH B
            analogWrite(3, HALF_STEP_POWER);   //Moves CH A
            analogWrite(11, HALF_STEP_POWER);   //Moves CH B

            motor_state++;
            break;
            
          case 2:
            digitalWrite(9, HIGH);  //DISABLE CH A
            //digitalWrite(8, LOW); //ENABLE CH B    //////////////////////
          
            //digitalWrite(13, HIGH);   //Sets direction of CH B ////////////////
            analogWrite(11, 255);   //Moves CH B
            
            motor_state++;
            break;

          case 3:
            digitalWrite(9, LOW);  //ENABLE CH A

            digitalWrite(12, LOW);   //Sets direction of CH A
            analogWrite(3, HALF_STEP_POWER);   //Moves CH A
            analogWrite(11, HALF_STEP_POWER);   //Moves CH B

            motor_state++;
            break;
            
          case 4:
            //digitalWrite(9, LOW);  //ENABLE CH A ////////////////
            digitalWrite(8, HIGH); //DISABLE CH B
          
            //digitalWrite(12, LOW);   //Sets direction of CH A //////////////////////
            analogWrite(3, 255);   //Moves CH A
            
            motor_state++;
            break;

          case 5:
            digitalWrite(8, LOW); //ENABLE CH B

            digitalWrite(13, LOW);   //Sets direction of CH B
            analogWrite(3, HALF_STEP_POWER);   //Moves CH A
            analogWrite(11, HALF_STEP_POWER);   //Moves CH B

            motor_state++;
            break;
            
          case 6:
            digitalWrite(9, HIGH);  //DISABLE CH A
            //digitalWrite(8, LOW); //ENABLE CH B //////////
          
            //digitalWrite(13, LOW);   //Sets direction of CH B ////////////
            analogWrite(11, 255);   //Moves CH B
            
            motor_state++;
            break;

          case 7:
            digitalWrite(9, LOW);  //ENABLE CH A

            digitalWrite(12, HIGH);   //Sets direction of CH A
            analogWrite(3, HALF_STEP_POWER);   //Moves CH A
            analogWrite(11, HALF_STEP_POWER);   //Moves CH B

            motor_state = 0;
            break;
            
        }
      }
      Serial.println(motor_pose);
    }
    else
    {
      digitalWrite(9, LOW);  //DISABLE CH A
      digitalWrite(8, LOW); //DISABLE CH B
      analogWrite(11, 0);
      analogWrite(3, 0);
    }
  }
}
