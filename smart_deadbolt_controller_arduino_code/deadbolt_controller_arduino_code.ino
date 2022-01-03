#include <LibPrintf.h>

#define IR_MOT_DET_PIN 7
#define PERSON_PRESENT_PIN 3
#define RELAY3 10
#define MOTOR_FORWARD A0
#define MOTOR_BACKWARD A1
#define GEAR_POT A3
#define MOTOR_SLEEP A2

#define LOCKED 1
#define UNLOCKED 0

#define SETUP_MODE false

#define LOCKED_POT_POS 210 // min value potentiometer must read for door to be "locked" (in degrees [0-270])
#define UNLOCKED_POT_POS  20 // max value potentiometer must read for door to be "locked" (in degrees [0-270])
#define IR_MOTDET_TO ((unsigned long) 20000)


unsigned long last_time_present = 0;
bool person_present = false;
bool person_present_recent_val = false;
short gear_pot_pos = 0;
bool deadbolt_current_state = false;
bool deadbolt_target_state = true;
bool motor_on = false;

void unlockDeadbolt(void)
{
  deadbolt_target_state = UNLOCKED;
  motor_on = true;
//  printf("Unlocking Deadbolt...\n");
  digitalWrite(MOTOR_BACKWARD, HIGH);
  digitalWrite(MOTOR_FORWARD, LOW);
}


void lockDeadbolt(void)
{
  deadbolt_target_state = LOCKED;
  motor_on = true;
//  printf("Locking Deadbolt...\n");
  digitalWrite(MOTOR_BACKWARD, LOW);
  digitalWrite(MOTOR_FORWARD, HIGH);
}

void stopDeadbolt(void)
{
  motor_on = false;
//  printf("Stopping Deadbolt Motor...\n");
  digitalWrite(MOTOR_BACKWARD, LOW);
  digitalWrite(MOTOR_FORWARD, LOW);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_MOT_DET_PIN, INPUT);
  pinMode(PERSON_PRESENT_PIN, OUTPUT);
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_BACKWARD, OUTPUT);
  pinMode(MOTOR_SLEEP, OUTPUT);
  // init value of gear potentiometer position 

  // enable motor controller
  digitalWrite(MOTOR_SLEEP, HIGH);
  if (!SETUP_MODE)
  {
    attachInterrupt(digitalPinToInterrupt(IR_MOT_DET_PIN), presenceISR, RISING);
  }
  Serial.begin(115200);
}

void loop() {
 
    if (person_present_recent_val != person_present)
    {
      
      person_present ? Serial.println("Person is now present!") : Serial.println("Person is no longer present.");
//      person_present ? unlockDeadbolt() : lockDeadbolt();
    }
    person_present_recent_val = person_present;
    if (millis() - last_time_present > IR_MOTDET_TO)
    {
      person_present = false;
    }
    digitalWrite(PERSON_PRESENT_PIN, person_present);

  gear_pot_pos = (analogRead(GEAR_POT) * 270.0) / 1024.0;

  if (!SETUP_MODE)
  {
    if (gear_pot_pos >= LOCKED_POT_POS)
    {
//      deadbolt_current_state = LOCKED;
      if (deadbolt_current_state != LOCKED)
      {
        if (deadbolt_current_state)
        stopDeadbolt();
        printf("Deadbolt is now locked\n");
        deadbolt_current_state = LOCKED;
      }
    }
    else if (gear_pot_pos <= UNLOCKED_POT_POS)
    {
//      deadbolt_current_state = UNLOCKED;
      if (deadbolt_current_state != UNLOCKED)
      {
        printf("Deadbolt is now unlocked\n");
        deadbolt_current_state = UNLOCKED;
          
        stopDeadbolt();
      }
    }
  //  else if (gear_pot_pos > UNLOCKED_POT_POS && gear_pot_pos < LOCKED_POT_POS){
  ////    if (deadboltCurrentState != MOVING)
  ////    {
  ////      printf("Deadbolt is now moving!\n");
  ////      deadboltCurrentState == MOVING;
  ////    }
  //  }

  }  
//
//  static volatile short new_deadbolt_pot_pos =  (analogRead(GEAR_POT) * 270.0) / 1024.0;
//  if ((new_deadbolt_pot_pos - gear_pot_pos > 1) || (new_deadbolt_pot_pos - gear_pot_pos < -1))
//  {
//    printf("gear potentiometer position [0-270deg] %d:\n", new_deadbolt_pot_pos); 
//  }
//  gear_pot_pos = new_deadbolt_pot_pos;
////  printf("raw potentiometer value [0-1024]: %ld\n", analogRead(GEAR_POT));
// 
//  printf("%d, %d\n", deadboltCurrentState, deadboltTargetState);
//  if (deadboltCurrentState != deadboltTargetState && deadboltCurrentState != MOVING)
//  {
//    if (deadboltTargetState == LOCKED)
//    {
//      lockDeadbolt();
//    }
//    else if (deadboltTargetState == UNLOCKED)
//    {
//      unlockDeadbolt();
//    }
//    else {
//      stopDeadbolt();
//    }
//  }
//
//
//  
  if (Serial.available() > 0)
  {
    char ui = Serial.read();
    if (ui == 's')
    {
      stopDeadbolt();
    }
    else if (ui == 'u')
    {
      unlockDeadbolt();
    }
    else if (ui == 'l')
    {
      lockDeadbolt();
    }
    else if (ui == 'p')
    {
      printf("gear potentiometer position [0-270deg] %d:\n", gear_pot_pos);
    }
    else if (ui == 'd')
    {
      printf("Deadbolt is currently %s!\n", deadbolt_current_state == LOCKED ? "locked" : "unlocked");
      printf("Deadbolt's target state is %s!\n", deadbolt_target_state == LOCKED ? "locked" : "unlocked");
      printf("Deadbolt is currently %s!\n", motor_on == true ? "moving" : "stopped");
    }
  }
  delay(10);
//  
//
//  
//
//
//  
//  
// 
//  

}

void presenceISR(void)
{
 
    Serial.println("Presence detected");
  person_present = true;
  last_time_present = millis();
    digitalWrite(PERSON_PRESENT_PIN, HIGH);
}
