#include <LibPrintf.h>
#include <FastLED.h>
#include <Bounce2.h>



//////////// MY SENSORS STUFF /////////////
// https://www.mysensors.org/download/sensor_api_20
// Enable debug prints
// #define MY_DEBUG

#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8

// Enable and select radio type attached
#define MY_RADIO_RF24

#include <MySensors.h>

// MySensors Sensor & Actuator IDs
#define DEADBOLT_ID 30 // ID of the deadbolt actuator that unlocks/locks door
#define MOTDET_ID 31  // ID of the motion sensor for presence detection in LR
#define DOOR_ID 32  // ID of the door sensor for detecting whether door is ajar / has been opened
#define KITCHEN_LIGHT_ID 33  // ID for a dummy sensor that will allow for creation of "light" entity in HASS
#define COUCH_CORNER_LIGHT_ID 34  // ID for a dummy sensor that will allow for creation of "light" entity in HASS
#define FISHTANK_LIGHT_ID 35  // ID for a dummy sensor that will allow for creation of "light" entity in HASS
#define DINING_TABLE_LIGHT_ID 36  // ID for a dummy sensor that will allow for creation of "light" entity in HASS

#define MOT_DET_PIN 2
#define PERSON_PRESENT_PIN 5
#define MOTOR_FORWARD A0
#define MOTOR_BACKWARD A1
#define GEAR_POT A3
#define MOTOR_SLEEP A2
#define DOOR_LIMIT_SWITCH A4
#define IR_PROXIMITY_SENSOR A5

#define LOCKED 1
#define UNLOCKED 0

#define SETUP_MODE 0

#define LOCKED_POT_POS 260 // min value potentiometer must read for door to be "locked" (in degrees [0-270])
#define UNLOCKED_POT_POS  80 // max value potentiometer must read for door to be "unlocked" (in degrees [0-270])
#define MOTOR_MIN_OVERSHOOT 5  // how many degrees further than the minimum locked/unlocked potentiometer pos must the motor turn the deadbolt until stopping
#define MOTOR_STALL_TIMEOUT 100 // how many ms motor must be "stalled" for until it is considered officially stalled
#define MOTOR_STALL_DIST 3 // min number of degrees that must be advanced within the motor stall timeout window  for it to be considered officially stalled

#define IR_MOTDET_TO ((unsigned long) 60000)

#define MANUAL_ASSIST_ACTIVATION_THRESHOLD 5 // how many units out of 1024 potentiometer must turn before deadbolt motion is interpreted as the deadbolt being manually turned (>=)

Bounce2::Button door_limit_switch_debouncer = Bounce2::Button();

unsigned long last_time_present = 0;
unsigned long time_motor_started = 0;
unsigned long last_time_pot_advanced = 0;
bool person_present = false;
bool person_present_recent_val = false;
short gear_pot_rot = 0;
short new_gear_pot_rot = 0;
bool deadbolt_current_state = UNLOCKED;
bool deadbolt_target_state = UNLOCKED;
bool motor_on = false;
short gear_pot_val_when_stopped;  // value read by potentiometer after deadbolt was finished being moved by motor [0-1024]
short gear_pot_val;
bool main_loop_entered = false;
bool door_open = false;
MyMessage deadbolt_msg(DEADBOLT_ID, V_ARMED);
MyMessage motdet_msg(MOTDET_ID, V_TRIPPED);
MyMessage door_msg(DOOR_ID, V_TRIPPED);

MyMessage kitchen_light_level_msg(KITCHEN_LIGHT_ID, V_PERCENTAGE );
MyMessage kitchen_light_state_msg(KITCHEN_LIGHT_ID, V_LIGHT );
MyMessage couch_corner_light_level_msg(COUCH_CORNER_LIGHT_ID, V_PERCENTAGE );
MyMessage couch_corner_light_state_msg(COUCH_CORNER_LIGHT_ID, V_LIGHT );
MyMessage fishtank_light_level_msg(FISHTANK_LIGHT_ID, V_PERCENTAGE );
MyMessage fishtank_light_state_msg(FISHTANK_LIGHT_ID, V_LIGHT );
MyMessage dining_table_light_level_msg(DINING_TABLE_LIGHT_ID, V_PERCENTAGE );
MyMessage dining_table_light_state_msg(DINING_TABLE_LIGHT_ID, V_LIGHT );

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Backdoor MySensors Node", "1.3");

    // Register all sensors to gw (they will be created as child devices)
    present(DEADBOLT_ID, S_LOCK);
    present(MOTDET_ID, S_MOTION);
    present(DOOR_ID, S_DOOR);
    present(KITCHEN_LIGHT_ID, S_DIMMER);
    present(COUCH_CORNER_LIGHT_ID, S_DIMMER);
    present(FISHTANK_LIGHT_ID, S_DIMMER);
    present(DINING_TABLE_LIGHT_ID, S_DIMMER);
}

// receive message from Gateway for deadbolt motor
void receive(const MyMessage &msg)
{
  printf("Message received for sensor #%d\n", msg.getSensor());
  if (msg.getType() == V_LOCK_STATUS)
  {
    // printf("Message received for sensor #%d\n", msg.getSensor());
    // if received LOCK command
    if (msg.getBool())
      lockDeadbolt();
    else
      unlockDeadbolt();
    printf(":D\n");
  }
}

void unlockDeadbolt(void)
{
  
  deadbolt_target_state = UNLOCKED;
  motor_on = true;
 printf("Unlocking Deadbolt...\n");
  digitalWrite(MOTOR_BACKWARD, HIGH);
  digitalWrite(MOTOR_FORWARD, LOW);
  send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  time_motor_started = millis();
  last_time_pot_advanced = millis();
}

void lockDeadbolt(void)
{
  deadbolt_target_state = LOCKED;
  motor_on = true;
 printf("Locking Deadbolt...\n");
  digitalWrite(MOTOR_BACKWARD, LOW);
  digitalWrite(MOTOR_FORWARD, HIGH);
  send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  time_motor_started = millis();
  last_time_pot_advanced = millis();
}

void stopDeadbolt(void)
{

  motor_on = false;
 printf("Stopping Deadbolt Motor. Action took %dms\n", millis()-time_motor_started);
  digitalWrite(MOTOR_BACKWARD, LOW);
  digitalWrite(MOTOR_FORWARD, LOW);
  wait(100); // pause to let everything settle to a rest
  gear_pot_val_when_stopped = analogRead(GEAR_POT);
  
}

void setup() {
  // put your setup code here, to run once:
  // pinMode(MOT_DET_PIN, INPUT);
  // pinMode(PERSON_PRESENT_PIN, OUTPUT);
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_BACKWARD, OUTPUT);
  pinMode(MOTOR_SLEEP, OUTPUT);
  // init value of gear potentiometer position 

  // enable motor controller
  digitalWrite(MOTOR_SLEEP, HIGH);
  // if (!SETUP_MODE)
  // {
  //   attachInterrupt(digitalPinToInterrupt(MOT_DET_PIN), presenceISR, RISING);
  // }

  // BUTTON SETUP 
  
  // SELECT ONE OF THE FOLLOWING :
  // 1) IF YOUR BUTTON HAS AN INTERNAL PULL-UP
  door_limit_switch_debouncer.attach( DOOR_LIMIT_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
  // 2) IF YOUR BUTTON USES AN EXTERNAL PULL-UP
  // button.attach( BUTTON_PIN, INPUT ); // USE EXTERNAL PULL-UP

  // DEBOUNCE INTERVAL IN MILLISECONDS
  door_limit_switch_debouncer.interval(1); 

  // INDICATE THAT THE HIGH STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
  door_limit_switch_debouncer.setPressedState(HIGH); 


  Serial.begin(115200);
  printf("Setup() complete! Serial connected.\n");
}

void loop() {
  if (!main_loop_entered)
  {
    time_motor_started = millis();
    stopDeadbolt();
    // update limit switch debouncer
    door_limit_switch_debouncer.update();
    door_open = door_limit_switch_debouncer.read() != 0;
      // enable motor controller
    digitalWrite(MOTOR_SLEEP, HIGH);
    Serial.print("\n\n\n< ENTERED MAIN LOOP >\n\n\n"); 
    main_loop_entered = true;
    send(deadbolt_msg.set(deadbolt_current_state?"1":"0"));
    send(motdet_msg.set("0"));
    send(door_msg.set(door_open ? "1" : "0"));
    send(kitchen_light_level_msg.set(100));
    send(kitchen_light_state_msg.set("1"));
    send(couch_corner_light_level_msg.set(100));
    send(couch_corner_light_state_msg.set("1"));
    send(fishtank_light_level_msg.set(100));
    send(fishtank_light_state_msg.set("1"));
    send(dining_table_light_level_msg.set(100));
    send(dining_table_light_state_msg.set("1"));
  }
  

  // if (!SETUP_MODE)
  // {
      //////////////////////////////////////
      // PRESENCE DETECTION CODE // 
      /////////////////////////////////////
  //     if (person_present_recent_val != person_present)
  //     {
        
  //       person_present ? Serial.println("Person is now present!") : Serial.println("Person is no longer present.");
  // //      person_present ? unlockDeadbolt() : lockDeadbolt();
  //     }
  //     person_present_recent_val = person_present;
  //     if (millis() - last_time_present > IR_MOTDET_TO)
  //     {
  //       person_present = false;
  //     }
  //     digitalWrite(PERSON_PRESENT_PIN, person_present);
  // }
     

      //////////////////////////////////////
      // DEADBOLT CONTROL CODE // 
      /////////////////////////////////////
      new_gear_pot_rot = (analogRead(GEAR_POT) * 270.0) / 1024.0;
      if ((deadbolt_target_state == LOCKED && new_gear_pot_rot > gear_pot_rot) 
      || (deadbolt_target_state == UNLOCKED && new_gear_pot_rot < gear_pot_rot))
      {
        last_time_pot_advanced = millis();
      }
       
       gear_pot_rot = new_gear_pot_rot;
      gear_pot_val = analogRead(GEAR_POT);
    if (!SETUP_MODE)
    {
      // if deadbolt has been locked
      if (gear_pot_rot >= LOCKED_POT_POS)
      {
        if (deadbolt_current_state != LOCKED)
        {
          if (deadbolt_current_state == UNLOCKED)
              { printf("Deadbolt is now locked\n"); }
          
          deadbolt_current_state = LOCKED;
          stopDeadbolt();
        }
        if (motor_on && deadbolt_target_state == LOCKED)
        {
          stopDeadbolt();
        }
      }
      // if deadbolt has been unlocked
      else if (gear_pot_rot <= UNLOCKED_POT_POS)
      {
        if (deadbolt_current_state != UNLOCKED)
        {
          if (deadbolt_current_state == LOCKED)
              { printf("Deadbolt is now unlocked\n"); }

          deadbolt_current_state = UNLOCKED;
          stopDeadbolt();
        }
        if (motor_on && deadbolt_target_state == UNLOCKED)
        {
          stopDeadbolt();
        }
      }

      // if motor has stalled
      if (motor_on && millis() - last_time_pot_advanced >= MOTOR_STALL_TIMEOUT)
      {
        printf("MOTOR STALLED!\n");
        if (deadbolt_target_state == LOCKED)
        {
          unlockDeadbolt();
        }
        else 
        {
          lockDeadbolt();
        }
      }

      // if door is open / ajar and deadbolt is currently unlocked and is trying to be unlocked
      if (door_open && motor_on && deadbolt_target_state == LOCKED && deadbolt_current_state == UNLOCKED)
      {
        printf("Door is ajar and deadbolt was attempting to lock... forcing unlock now!\n");
        unlockDeadbolt();
      }
      

      // update limit switch debouncer
      door_limit_switch_debouncer.update();
      
      // limit switch's state has changed
      if ( door_limit_switch_debouncer.changed() ) { 
        
        // if limit switch is NOT "pressed" ( == 0) then door has been opened
        door_open = door_limit_switch_debouncer.read() != 0;

        // send updated door state msg
        send(door_msg.set(door_open ? "1" : "0"));

        // if door has just been opened
        if (door_open)
        {
          printf("Door was just opened!\n");
        }
        // if door has just been closed
        else 
        {
          printf("Door was just closed!\n");
          lockDeadbolt();
        }

      }

      if(digitalRead(IR_PROXIMITY_SENSOR) == HIGH && !door_open)
      {

      }

      
      // // if deadbolt has been moved manually 
      // if (!motor_on && 
      //     (  gear_pot_val - gear_pot_val_when_stopped <= -MANUAL_ASSIST_ACTIVATION_THRESHOLD
      //     || gear_pot_val - gear_pot_val_when_stopped >= MANUAL_ASSIST_ACTIVATION_THRESHOLD))
      // {
      //   printf("DEADBOLT MOVED MANUALLY \n");
      //     // if currently locked
      //     if (deadbolt_current_state == LOCKED 
      //     // and the deadbolt has been turned in the unlock direction
      //     && gear_pot_val - gear_pot_val_when_stopped <= -MANUAL_ASSIST_ACTIVATION_THRESHOLD)
      //     {
      //         // wait(1000);
      //         // unlockDeadbolt();
      //         printf("manual UNLOCK assist activated\n");
      //     }
      //     else if (deadbolt_current_state == UNLOCKED 
      //     // and the deadbolt has been turned in the lock direction
      //     && gear_pot_val - gear_pot_val_when_stopped >= MANUAL_ASSIST_ACTIVATION_THRESHOLD)
      //     {
      //         // wait(1000);
      //         // lockDeadbolt();
      //         printf("manual LOCK assist activated\n");
      //     }
      // }



    }  

  
    //////////////////////////////////////
    // DEBUG & TESTING W/ SERIAL CODE // 
    /////////////////////////////////////
  if (Serial.available() > 0)
  {
    char ui = Serial.read();
    if (ui == 's')
    {
      stopDeadbolt();
    }
    // else if (ui == '<')
    // {
    //   printf("moving deadbolt left... ");
    //   lockDeadbolt();
    //   delay(250);
    //   stopDeadbolt();
    //   printf("stopped deadbolt.\n");

    // }
    // else if (ui == '>')
    // {
    //   printf("moving deadbolt right... ");
    //   unlockDeadbolt();
    //   delay(250);
    //   stopDeadbolt();
    //   printf("stopped deadbolt.\n");
    // }
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
      printf("gear potentiometer rotation [0-270deg]: %d ; raw value: %d\n", gear_pot_rot, gear_pot_val);
    }
    else if (ui == 'd')
    {
      printf("Deadbolt is currently %s!\n", deadbolt_current_state == LOCKED ? "locked" : "unlocked");
      printf("Deadbolt's target state is %s!\n", deadbolt_target_state == LOCKED ? "locked" : "unlocked");
      printf("Deadbolt is currently %s!\n", motor_on == true ? "moving" : "stopped");
    }
  }
  wait(10);
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

// void presenceISR(void)
// {
 
//     Serial.println("Presence detected");
//   person_present = true;
//   last_time_present = millis();
//     digitalWrite(PERSON_PRESENT_PIN, HIGH);
// }
