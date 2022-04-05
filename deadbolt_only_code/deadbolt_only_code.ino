#include <LibPrintf.h>
#include <Bounce2.h>
// #inc lude <SharpIR.h>

#include <SoftwareSerial.h>


#undef RFID_ENABLED

#ifdef RFID_ENABLED
  #include <SPI.h>
  #include <MFRC522.h>
#endif
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Arduino Pro Mini Pinout Resrvations ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// [ TX   ]   Pin 0  - 
// [ RX   ]   Pin 1  - 
// [ INT  ]   Pin 2  - Limit Switch Digital Input
// [ INT  ] ~ Pin 3  - 
// [      ]   Pin 4  - Motor Stalled Diagnostic LED
// [      ] ~ Pin 5  - Door Open Diagnostic LED
// [      ] ~ Pin 6  - Deadbolt Toggle Button Digital Input
// [      ]   Pin 7  - RF24 CE
// [      ]   Pin 8  - RF24 CS
// [      ] ~ Pin 9  - Hand Detected Diagnostic LED
// [ SS   ] ~ Pin 10 - 
// [ MOSI ] ~ Pin 11 - RF24
// [ MISO ]   Pin 12 - RF24
// [ SCK  ]   Pin 13 - RF24
// [      ]   Pin A0 - Motor Forward Digital Output
// [      ]   Pin A1 - Motor Backward Digital Output
// [      ]   Pin A2 - Motor Driver Sleep Digital Output (HIGH = Enable)
// [      ]   Pin A3 - Potentiometer Analog Input
// [      ]   Pin A4 - Ultrasonic Distance Sensor TRIG
// [      ]   Pin A5 - Ultrasonic Distance Sensor ECHO
// [      ]   Pin A6 - 
// [      ]   Pin A7 - 


//SoftwareSerial mySerial(2, 3); // RX, TX

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////


//////////// MY SENSORS STUFF /////////////
// https://www.mysensors.org/download/sensor_api_20
// Enable debug prints
// #define MY_DEBUG
//#define MY_NODE_ID  2 
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
#define RFID_ID 37

// #define MOT_DET_PIN 2
// #define PERSON_PRESENT_PIN 5
#define MOTOR_FORWARD A0
#define MOTOR_BACKWARD A1
#define GEAR_POT A3
#define MOTOR_SLEEP A2
#define DOOR_LIMIT_SWITCH 2
// #define IR_DISTANCE_SENSOR 
#define DEADBOLT_TOGGLE_BUTTON 6
#define ULTRASONIC_TRIG A4
#define ULTRASONIC_ECHO A5
#define DOOR_OPEN_LED 5
#define MOTOR_STALLED_LED 4 
#define HAND_DETECTED_LED 9



#define LOCKED 1
#define UNLOCKED 0


#define SETUP_MODE 0
#define ULTRASONIC_TRIALS 3
#define LOCKED_POT_POS 190 // min value potentiometer must read for door to be "locked" (in degrees [0-270])
#define UNLOCKED_POT_POS  20 // max value potentiometer must read for door to be "unlocked" (in degrees [0-270])
#define MOTOR_MIN_OVERSHOOT 5  // how many degrees further than the minimum locked/unlocked potentiometer pos must the motor turn the deadbolt until stopping
#define MOTOR_STALL_TIMEOUT 200 // how many ms motor must be "stalled" for until it is considered officially stalled
#define MOTOR_STALL_DIST 1 // min number of degrees that must be advanced within the motor stall timeout window  for it to be considered officially stalled
#define HAND_PRESENT_MAX_DIST 75 // in cm
#define IR_MOTDET_TO ((unsigned long) 60000)
#define HEARTBEAT_INTERVAL 1000 // in ms
#define MANUAL_ASSIST_ACTIVATION_THRESHOLD 5 // how many units out of 1024 potentiometer must turn before deadbolt motion is interpreted as the deadbolt being manually turned (>=)
#define KEEP_UNLOCKED_DUR 15000  // in ms, how long should deadbolt stay unlocked for before auto re-locking after being unlocked 

//Bounce2::Button door_limit_switch_debouncer = Bounce2::Button();
//Bounce2::Button deadbolt_toggle_button = Bounce2::Button();
#ifdef RFID_ENABLED
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);  // Create MFRC522 instance.
#endif
// SharpIR hand_distance_sensor(GP2Y0 A21YK0F_MODEL_CODE, IR_DISTANCE_SENSOR);

unsigned long last_time_heartbeat = 0;
unsigned long last_time_hand_detected = 0;
unsigned long last_time_door_closed = 0;
unsigned long last_time_unlocked = 0;
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
bool motor_stalled = false;
short gear_pot_val_when_stopped;  // value read by potentiometer after deadbolt was finished being moved by motor [0-1024]
short gear_pot_val;
bool main_loop_entered = false;
bool door_open = false;
bool door_limit_switch_pressed = false;
bool rfid_accepted = false;
unsigned int hand_distance = 1000;  // in cm ; set to high starting value so nothing gets triggered at boot
unsigned long dur; // variable for the duration of sound wave travel
bool rfid_unlocking_enabled = true;
static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  
MyMessage deadbolt_msg(DEADBOLT_ID, V_LOCK_STATUS);
MyMessage motdet_msg(MOTDET_ID, V_TRIPPED);
MyMessage door_msg(DOOR_ID, V_TRIPPED);
// MyMessage rfid_msg(RFID_ID, V_LOCK_STATUS);
//
//MyMessage kitchen_light_level_msg(KITCHEN_LIGHT_ID, V_PERCENTAGE );
//MyMessage kitchen_light_state_msg(KITCHEN_LIGHT_ID, V_LIGHT );
//MyMessage couch_corner_light_level_msg(COUCH_CORNER_LIGHT_ID, V_PERCENTAGE );
//MyMessage couch_corner_light_state_msg(COUCH_CORNER_LIGHT_ID, V_LIGHT );
//MyMessage fishtank_light_level_msg(FISHTANK_LIGHT_ID, V_PERCENTAGE );
//MyMessage fishtank_light_state_msg(FISHTANK_LIGHT_ID, V_LIGHT );
//MyMessage dining_table_light_level_msg(DINING_TABLE_LIGHT_ID, V_PERCENTAGE );
//MyMessage dining_table_light_state_msg(DINING_TABLE_LIGHT_ID, V_LIGHT );


void before() {
  #ifdef RFID_ENABLED
  // Make sure MFRC is disabled from the SPI bus
  pinMode(RFID_SS_PIN, OUTPUT);
  digitalWrite(RFID_SS_PIN, HIGH);
  #endif
}

void presentation()
{
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Backdoor MySensors Node", "22-04-03");

    // Register all sensors to gw (they will be created as child devices)
    present(DEADBOLT_ID, S_LOCK);
    present(MOTDET_ID, S_MOTION);
    present(DOOR_ID, S_DOOR);
//    present(KITCHEN_LIGHT_ID, S_DIMMER);
//    present(COUCH_CORNER_LIGHT_ID, S_DIMMER);
//    present(FISHTANK_LIGHT_ID, S_DIMMER);
//    present(DINING_TABLE_LIGHT_ID, S_DIMMER);
    // present(RFID_ID, S_LOCK);
}

// receive message from Gateway for deadbolt motor
void receive(const MyMessage &msg)
{
  printf("Message received for sensor #%d\n", msg.getSensor());
  if (msg.getType() == V_LOCK_STATUS)
  {
    bool lock_status_recvd = msg.getBool();
    // printf("Message received for sensor #%d\n", msg.getSensor());
    // if received LOCK command
    // if (msg.getSensor() == RFID_ID)
    // {
    //   if (!lock_status_recvd) {printf("Access granted has been terminated by HASS!\n");}
    // }   
    if (msg.getSensor() == DEADBOLT_ID)
    {
      if (lock_status_recvd)
        lockDeadbolt();
      else
        unlockDeadbolt();
      printf(":D\n");
    }
    
  }
}

void unlockDeadbolt(void)
{
  // if not already unlocked
//  if (gear_pot_rot > UNLOCKED_POT_POS)
if (deadbolt_target_state != UNLOCKED || deadbolt_current_state != UNLOCKED)
  {
    if (!motor_on || deadbolt_target_state != UNLOCKED)
    {
     motor_on = true;
     printf("Unlocking Deadbolt...\n");
      digitalWrite(MOTOR_BACKWARD, HIGH);
      digitalWrite(MOTOR_FORWARD, LOW);
      time_motor_started = millis();
      last_time_pot_advanced = millis(); 
    }
    deadbolt_target_state = UNLOCKED;
    last_time_unlocked = millis();
    send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  }
  else
  {
   deadbolt_target_state = UNLOCKED; 
  last_time_unlocked = millis();
//    printf("unlockDeadbolt(): Deadbolt already unlocked!\n");
    stopDeadbolt();
//  send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  }
  
}

void lockDeadbolt(void)
{
  if (deadbolt_target_state != LOCKED || deadbolt_current_state != LOCKED)
  {
    if (!motor_on || deadbolt_target_state != LOCKED)
    {
      motor_on = true;
     printf("Locking Deadbolt...\n");
      digitalWrite(MOTOR_BACKWARD, LOW);
      digitalWrite(MOTOR_FORWARD, HIGH);
      time_motor_started = millis();
      last_time_pot_advanced = millis();
      send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
    }
    
    deadbolt_target_state = LOCKED;
  }
  else 
  {
   deadbolt_target_state = LOCKED; 
//    printf("lockDeadbolt(): Deadbolt already locked!\n");
    stopDeadbolt();
  send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  }
  
}

void stopDeadbolt(void)
{
if (motor_on)
{
  motor_stalled = false;
  digitalWrite(MOTOR_STALLED_LED, motor_stalled);
  motor_on = false;
 printf("Stopping Deadbolt Motor. Action took %dms\n", millis()-time_motor_started);
  digitalWrite(MOTOR_BACKWARD, LOW);
  digitalWrite(MOTOR_FORWARD, LOW);
  
  send(deadbolt_msg.set(deadbolt_target_state?"1":"0"));
  wait(100); // pause to let everything settle to a rest
}
  
  gear_pot_val_when_stopped = analogRead(GEAR_POT);
  
}

void setup() {

  
  Serial.begin(115200);
  // put your setup code here, to run once:
  // pinMode(MOT_DET_PIN, INPUT);
  // pinMode(PERSON_PRESENT_PIN, OUTPUT);
  pinMode(MOTOR_FORWARD, OUTPUT);
  pinMode(MOTOR_BACKWARD, OUTPUT);
  pinMode(MOTOR_SLEEP, OUTPUT);
  pinMode(DOOR_OPEN_LED, OUTPUT);
  pinMode(MOTOR_STALLED_LED, OUTPUT);
  pinMode(HAND_DETECTED_LED, OUTPUT);
  digitalWrite(DOOR_LIMIT_SWITCH, HIGH);
  
attachInterrupt(digitalPinToInterrupt(DOOR_LIMIT_SWITCH), door_limit_switch_irq_handler, CHANGE); 

  // enable motor controller
  digitalWrite(MOTOR_SLEEP, HIGH);
  // if (!SETUP_MODE)
  // {
  //   attachInterrupt(digitalPinToInterrupt(MOT_DET_PIN), presenceISR, RISING);
  // }

//  door_limit_switch_debouncer.attach( DOOR_LIMIT_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
//  deadbolt_toggle_button.attach( DEADBOLT_TOGGLE_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP

  // DEBOUNCE INTERVAL IN MILLISECONDS
//  door_limit_switch_debouncer.interval(10); 
//  deadbolt_toggle_button.interval(1); 

  // INDICATE THAT THE HIGH STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
//  door_limit_switch_debouncer.setPressedState(HIGH); 
//  deadbolt_toggle_button.setPressedState(HIGH); 

  #ifdef RFID_ENABLED
  // RFID module
  SPI.begin();         // Initiate  SPI bus
  mfrc522.PCD_Init();  // Initiate MFRC522
  pinMode(RFID_RST_PIN, OUTPUT);
  pinMode(RFID_SS_PIN, OUTPUT);
  #endif
pinMode(ULTRASONIC_TRIG, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(ULTRASONIC_ECHO, INPUT); // Sets the echoPin as an INPUT
  
//mySerial.begin(1200);
  printf("Setup() complete! Serial connected.\n");
}

void calc_hand_distance()
{
  hand_distance = 0;
  for (int i=0; i < ULTRASONIC_TRIALS; i++)
  {
    // Clears the trigPin condition
    digitalWrite(ULTRASONIC_TRIG, LOW);
    delayMicroseconds(2);
    // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    digitalWrite(ULTRASONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRASONIC_TRIG, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    dur = pulseIn(ULTRASONIC_ECHO, HIGH);
    // Calculating the distance
    hand_distance += dur * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  }
  hand_distance = hand_distance / ULTRASONIC_TRIALS;

//  printf("Ultrasonic Sensor Reads:  %d cm )\n", hand_distance);
}

#ifdef RFID_ENABLED
void read_rfid(void) {
  
  digitalWrite(MY_RF24_CS_PIN, HIGH);
//  digitalWrite(RFID_SS_PIN, LOW);
//  wait(100);
  // Look for new cards
  if (!mfrc522.PICC_IsNewCardPresent()) {
    rfid_accepted = false;
    digitalWrite(RFID_SS_PIN, HIGH);
//    digitalWrite(MY_RF24_CS_PIN, LOW);
//    printf("rfid goodbye ONE...\n");
    return;
  }
  // Select one of the cards
  if (!mfrc522.PICC_ReadCardSerial()) {
    rfid_accepted = false;
    digitalWrite(RFID_SS_PIN, HIGH);
//    digitalWrite(MY_RF24_CS_PIN, LOW);
//    printf("rfid goodbye TWO...\n");
    return;
  }
  //Show UID on serial monitor
  Serial.print("UID tag :");
  String content = "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
    Serial.print(mfrc522.uid.uidByte[i], HEX);
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  Serial.println();
  Serial.print("Message : ");
  content.toUpperCase();
  if (content.substring(1) == "F6 93 AA 13" || content.substring(1) == "04 23 84 E2 50 60 80")  //change here the UID of the card/cards that you want to give access
  {
    rfid_accepted = true;
    // send(rfid_msg.set(rfid_accepted ? "1" : "0"));
    printf("Authorized access");
    
  }

  else {
    rfid_accepted = false;
  }
  digitalWrite(RFID_SS_PIN, HIGH);
//    digitalWrite(MY_RF24_CS_PIN, LOW);
}
#endif

void door_limit_switch_irq_handler()
{
  printf("irq : limit switch %spressed\n", digitalRead(DOOR_LIMIT_SWITCH) == HIGH ? "NOT " : "");
  last_interrupt_time = 0;
  interrupt_time = millis();
  // If interrupts come faster than 10ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 10) 
  {
    door_limit_switch_pressed = digitalRead(DOOR_LIMIT_SWITCH) == LOW;
  }
  last_interrupt_time = interrupt_time;
  
}


void loop() {
  if (!main_loop_entered)
  {
    time_motor_started = millis();
    stopDeadbolt();
    // update limit switch debouncer
//    door_limit_switch_debouncer.update();
//    door_open = door_limit_switch_debouncer.read() != 0;
      // enable motor controller
    digitalWrite(MOTOR_SLEEP, HIGH);
    Serial.print("\n\n\n< ENTERED MAIN LOOP >\n\n\n"); 
    main_loop_entered = true;
    send(deadbolt_msg.set(deadbolt_current_state?"1":"0"));
    send(motdet_msg.set("0"));
    send(door_msg.set(door_open ? "1" : "0"));
//    send(kitchen_light_level_msg.set(100));
//    send(kitchen_light_state_msg.set("1"));
//    send(couch_corner_light_level_msg.set(100));
//    send(couch_corner_light_state_msg.set("1"));
//    send(fishtank_light_level_msg.set(100));
//    send(fishtank_light_state_msg.set("1"));
//    send(dining_table_light_level_msg.set(100));
//    send(dining_table_light_state_msg.set("1"));
    // send(rfid_msg.set(access_granted ? "1" : "0"));
  }
  

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
        motor_stalled = true;
        digitalWrite(MOTOR_STALLED_LED, motor_stalled);
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

      #ifdef RFID_ENABLED
      // activate rfid reader and run RFID scanner logic
      read_rfid();

      // if valid rfid device/tag presented then unlock the deadbolt
      if (rfid_accepted)
      {
        printf("deadbolt unlocked via RFID \n");
        rfid_accepted = false;
        unlockDeadbolt();
      }
      #endif

      // update limit switch debouncer
//      door_limit_switch_debouncer.update();
      
      // limit switch's state has changed
//      if ( door_limit_switch_debouncer.changed() ) { 
//        
//        // if limit switch is NOT "pressed" ( == 0) then door has been opened
//         door_open = door_limit_switch_debouncer.read() != 0;
//
//        // after old wire snapped, using normal closed wire instead now
////        door_open = door_limit_switch_debouncer.read() == 0;
//        digitalWrite(DOOR_OPEN_LED, door_open);
//        // send updated door state msg
//        send(door_msg.set(door_open ? "1" : "0"));
//
//        // if door has just been opened
//        if (door_open)
//        {
//          printf("Door was just opened!\n");
//        }
//        // if door has just been closed
//        else 
//        {
//          last_time_door_closed = millis();
//          printf("Door was just closed!\n");
//          lockDeadbolt();
//        }
//
//      }


  if ( door_limit_switch_pressed == door_open) { 
        
        // if limit switch is NOT "pressed" ( == 0) then door has been opened
         door_open = !door_limit_switch_pressed;

        // after old wire snapped, using normal closed wire instead now
//        door_open = door_limit_switch_debouncer.read() == 0;
        digitalWrite(DOOR_OPEN_LED, door_open);
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
          last_time_door_closed = millis();
          printf("Door was just closed!\n");
          lockDeadbolt();
        }

      }

//      
//
//      
//      // if door is open / ajar and deadbolt is currently unlocked and is trying to be locked
//      if (door_open && motor_on && deadbolt_target_state == LOCKED && deadbolt_current_state == UNLOCKED)
//      {
//        printf("Door is ajar and deadbolt was attempting to lock... forcing unlock now!\n");
//        unlockDeadbolt();
//      }

//      deadbolt_toggle_button.update();
//      
//      if ( deadbolt_toggle_button.changed() ) { 
//        
//        // if button has been pressed
//        if (deadbolt_toggle_button.read() == 0);
//        {
//          if (deadbolt_target_state == LOCKED)
//            unlockDeadbolt();
//          else
//            lockDeadbolt();
//        }
//
//      }

      // activate ultrasonic sensor to read hand distance 
      if (millis() - last_time_hand_detected > 1000)
      { 
        calc_hand_distance(); 
        
        // if hand detected
        if(hand_distance < HAND_PRESENT_MAX_DIST)
        {
          digitalWrite(HAND_DETECTED_LED, HIGH);
          last_time_hand_detected = millis();
          if (millis() - last_time_door_closed > 3000)
          {
            if (motor_on && deadbolt_target_state == LOCKED)
            {
              printf("hand detected @ %d cm BUT unlocking deadbolt is prohibited right now\n", hand_distance);
            }
            else
            {
              printf("hand detected @ %d cm ; sending command to unlock deadbolt\n", hand_distance);
              // unlock deadbolt
              unlockDeadbolt();
            }
            
          }
        }
        else 
        {
          digitalWrite(HAND_DETECTED_LED, LOW);
        }

      }
       
      
 // if it is time for door to auto re-lock and it is still unlocked and door is shut and deadbolt isnt moving
       if (millis() - last_time_unlocked > KEEP_UNLOCKED_DUR && !door_open && !motor_on && deadbolt_current_state == UNLOCKED)
       {
        printf("Auto re-locking deadbolt!\n");
        lockDeadbolt();
       }

     

  
    }  

    //// COMMUNICATE WITH AUTHENTICATION ARDUINO /////

     if (Serial.available() > 0)
 {
   char rx = Serial.read();
  //  bool authenticated = (rx == '1');
    printf("Received %d ('%c') from auth arduino!\n", rx,rx);
    // if rfid or fingerprint authorized
   if (rx == 'r' || rx == 'f') 
   {
     printf("Authorized using %s\n", rx == 'r' ? "RFID" : "Fingerprint");
     unlockDeadbolt();
   }

 }

 if (!motor_on && millis() - last_time_heartbeat > HEARTBEAT_INTERVAL)
 {
  
  send(deadbolt_msg.set(deadbolt_current_state?"1":"0"));
  last_time_heartbeat = millis();
 }

  
    //////////////////////////////////////
    // DEBUG & TESTING W/ SERIAL CODE // 
    /////////////////////////////////////
//  if (Serial.available() > 0)
//  {
//    char ui = Serial.read();
//    if (ui == 's')
//    {
//      stopDeadbolt();
//    }
//
//    else if (ui == 'u')
//    {
//      unlockDeadbolt();
//    }
//    else if (ui == 'l')
//    {
//      lockDeadbolt();
//    }
//    else if (ui == 'p')
//    {
//      printf("gear potentiometer rotation [0-270deg]: %d ; raw value: %d\n", gear_pot_rot, gear_pot_val);
//    }
//    else if (ui == 'd')
//    {
//      printf("Deadbolt is currently %s!\n", deadbolt_current_state == LOCKED ? "locked" : "unlocked");
//      printf("Deadbolt's target state is %s!\n", deadbolt_target_state == LOCKED ? "locked" : "unlocked");
//      printf("Deadbolt is currently %s!\n", motor_on == true ? "moving" : "stopped");
//    }
//    // test out hand distance sensor for 5 seconds
//    else if (ui == 'h' && !motor_on)
//    {
//      
//    }
//  }
 
}
