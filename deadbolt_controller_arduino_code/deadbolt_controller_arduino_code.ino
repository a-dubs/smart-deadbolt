#include <LibPrintf.h>
#include <Bounce2.h>
// #inc lude <SharpIR.h>
#include <FastLED.h>

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
// [ INT  ] ~ Pin 3  - Deadbolt Toggle Button Digital Input
// [      ]   Pin 4  - 
// [      ] ~ Pin 5  - 
// [      ] ~ Pin 6  - 
// [      ]   Pin 7  - RF24 CE
// [      ]   Pin 8  - RF24 CS
// [      ] ~ Pin 9  - LED Strip Data Pin
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

#define MOTOR_FORWARD A0
#define MOTOR_BACKWARD A1
#define GEAR_POT A3
#define MOTOR_SLEEP A2
#define DOOR_LIMIT_SWITCH 2
// #define IR_DISTANCE_SENSOR
#define DEADBOLT_TOGGLE_BUTTON 3
#define ULTRASONIC_TRIG A4
#define ULTRASONIC_ECHO A5
#define LED_STRIP_DATA 9
#define MY_RF24_CE_PIN 7
#define MY_RF24_CS_PIN 8

////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//////////// MY SENSORS STUFF /////////////
// https://www.mysensors.org/download/sensor_api_20
// Enable debug prints
// #define MY_DEBUG
//#define MY_NODE_ID  2


// Enable and select radio type attached
#define MY_RADIO_RF24

#include <MySensors.h>

// MySensors Sensor & Actuator IDs
#define DEADBOLT_ID 30           // ID of the deadbolt actuator that unlocks/locks door
#define MOTDET_ID 31             // ID of the motion sensor for presence detection in LR
#define DOOR_ID 32               // ID of the door sensor for detecting whether door is ajar / has been opened

#define RFID_ID 37

// #define MOT_DET_PIN 2
// #define PERSON_PRESENT_PIN 5


#define LOCKED 1
#define UNLOCKED 0
#define SETUP_MODE 0
#define ULTRASONIC_TRIALS 1
#define LOCKED_POT_POS 220      // min value potentiometer must read for door to be "locked" (in degrees [0-270])
#define UNLOCKED_POT_POS 40      // max value potentiometer must read for door to be "unlocked" (in degrees [0-270])
#define MOTOR_MIN_OVERSHOOT 5    // how many degrees further than the minimum locked/unlocked potentiometer pos must the motor turn the deadbolt until stopping
#define MOTOR_STALL_TIMEOUT 200  // how many ms motor must be "stalled" for until it is considered officially stalled
#define MOTOR_STALL_DIST 1       // min number of degrees that must be advanced within the motor stall timeout window  for it to be considered officially stalled
#define HAND_PRESENT_MAX_DIST 50 // in cm
#define IR_MOTDET_TO ((unsigned long)60000)
#define HEARTBEAT_INTERVAL 1000              // in ms
#define MANUAL_ASSIST_ACTIVATION_THRESHOLD 5 // how many units out of 1024 potentiometer must turn before deadbolt motion is interpreted as the deadbolt being manually turned (>=)
#define KEEP_UNLOCKED_DUR 15000              // in ms, how long should deadbolt stay unlocked for before auto re-locking after being unlocked
#define DOOR_FULLY_OPEN_MIN_DUR 2000
#define MOTOR_MAX_TIME_ON 1000  // how long the motor can continuously run for before being considered stalled

#define ULTRASONIC_ENABLED false
// Bounce2::Button door_limit_switch_debouncer = Bounce2::Button();
//Bounce2::Button deadbolt_toggle_button = Bounce2::Button();
#ifdef RFID_ENABLED
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN); // Create MFRC522 instance.
#endif
// SharpIR hand_distance_sensor(GP2Y0 A21YK0F_MODEL_CODE, IR_DISTANCE_SENSOR);

//////////////////////////////////////////////////
///////////////// FAST LED STUFF /////////////////
//////////////////////////////////////////////////

#define NUM_LEDS 8
CRGB leds[NUM_LEDS];
unsigned long leds_times_last_updated[NUM_LEDS];
#define TEMP_LEDS_DUR_ON 2000 

/*  DIAGNOSTIC LED STRIP COLOR CODE MEANINGS
+------------------------+-----------------------+
| Purpose                | Color Codes           |
+------------------------+-----------------------+
| Door State             | Red: Ajar             |
|                        | White: Shut           |
+------------------------+-----------------------+
| Deadbolt Current State | Green: Unlocked       |
|                        | Orange: Locked        |
+------------------------+-----------------------+
| Deadbolt Target State  | Green: Unlocked       |
|                        | Orange: Locked        |
+------------------------+-----------------------+
| Hand Detected          | Blue: Hand Detected   |
|                        | Off: Nothing Detected |
+------------------------+-----------------------+
| Authentication         | Red: RFID             |
|                        | Blue: Fingerprint     |
|                        | Off: None             |
+------------------------+-----------------------+
| Motor Warnings         | Red: Stalled          |
|                        | Off: None             |
+------------------------+-----------------------+
| Motor State            | Cyan: Motor on        |
|                        | Off: Motor off        |
+------------------------+-----------------------+
| Deadbolt Last Trigger  | HASS: Cyan            |
|                        | Auth: Magenta         |
|                        | Button: Red           |
|                        | Hand: Blue            |
+------------------------+-----------------------+
| RESERVED               | N/A                   |
+------------------------+-----------------------+
| RESERVED               | N/A                   |
+------------------------+-----------------------+
*/

#define MY_RED CRGB(255, 0, 0)
#define MY_GREEN CRGB(0, 200, 0)
#define MY_BLUE CRGB(0, 0, 200)
#define MY_CYAN CRGB(0, 200, 255)
#define MY_MAGENTA CRGB(255, 0, 200)
#define MY_ORANGE CRGB(150, 70, 0)
#define MY_WHITE CRGB(255, 255, 255)
#define MY_OFF CRGB(0,0,0)

#define DOOR_AJAR_COLOR MY_RED
#define DOOR_SHUT_COLOR MY_WHITE
#define DB_STATE_UNLOCKED_COLOR MY_GREEN
#define DB_STATE_LOCKED_COLOR MY_ORANGE
#define AUTH_RFID_COLOR MY_RED
#define AUTH_FINGERPRINT_COLOR MY_BLUE
#define AUTH_CONNECTED_COLOR MY_WHITE
#define AUTH_DISCONNECTED_COLOR MY_OFF
#define HAND_DET_COLOR MY_BLUE
#define HAND_NOT_DET_COLOR MY_OFF
#define MOTOR_WARN_STALL_COLOR MY_RED
#define MOTOR_WARN_NONE_COLOR MY_OFF
#define MOTOR_STATE_ON_COLOR MY_CYAN
#define MOTOR_STATE_OFF_COLOR MY_OFF 
#define TRIG_HASS_COLOR MY_CYAN
#define TRIG_AUTH_COLOR MY_MAGENTA
#define TRIG_BUTTON_COLOR MY_RED
#define TRIG_HAND_COLOR MY_BLUE


#define LEDS_DOOR_STATE 0
#define LEDS_DB_C_STATE 1 // CURRENT STATE
#define LEDS_DB_T_STATE 2 // TARGET STATE
#define LEDS_HAND_DET 3
#define LEDS_AUTH 4
#define LEDS_MOTOR_WARN 5
#define LEDS_MOTOR_STATE 6
#define LEDS_DB_LAST_TRIG 7
#define LEDS_RESERVED_1 8
#define LEDS_RESERVED_2 9

bool is_led_persistent[NUM_LEDS] = {
    true,    // LEDS_DOOR_STATE 0
    true,    // LEDS_DB_C_STATE 1 // CURRENT STATE
    true,    // LEDS_DB_T_STATE 2 // TARGET STATE
    true,    // LEDS_HAND_DET 3
    false,    // LEDS_AUTH 4
    false,    // LEDS_MOTOR_WARN 5
    true,    // LEDS_MOTOR_STATE 6
    false    // LEDS_DB_LAST_TRIG 7
    };

void set_led(int led_index, CRGB color)
{
    leds[led_index] = color;
    FastLED.show();
    leds_times_last_updated[led_index] = millis();
}

void check_to_clear_leds()
{
    for (int i=0; i < NUM_LEDS; i++)
    {
        if (!is_led_persistent[i] && leds[i] != MY_OFF && (millis() - leds_times_last_updated[i] > TEMP_LEDS_DUR_ON))
        {
            leds[i] = MY_OFF;
            printf("turned off led at index %d\n", i);
            FastLED.show();
        }
    }
    
}

void set_led_strip(CRGB color)
{
    fill_solid(&(leds[0]), NUM_LEDS, color);
    FastLED.show();
}

//////////////////////////////////////////////////
//////////////////////////////////////////////////
//////////////////////////////////////////////////


unsigned long last_time_heartbeat = 0;
unsigned long last_time_hand_detected = 0;
unsigned long last_time_door_closed = 0;
unsigned long last_time_door_opened = 0;
unsigned long last_time_unlocked = 0;
unsigned long last_time_present = 0;
unsigned long time_motor_started = 0;
unsigned long last_time_pot_advanced = 0;
bool door_was_fully_opened = false;
bool person_present = false;
bool person_present_recent_val = false;
short gear_pot_rot = 0;
short new_gear_pot_rot = 0;
bool deadbolt_current_state = UNLOCKED;
bool deadbolt_target_state = UNLOCKED;
bool motor_on = false;
bool motor_stalled = false;
short gear_pot_val_when_stopped; // value read by potentiometer after deadbolt was finished being moved by motor [0-1024]
short gear_pot_val;
bool main_loop_entered = false;
bool door_open = false;
bool button_pressed = false;
bool door_limit_switch_pressed = false;
bool rfid_accepted = false;
unsigned int hand_distance = 1000; // in cm ; set to high starting value so nothing gets triggered at boot
unsigned long dur;                 // variable for the duration of sound wave travel
bool rfid_unlocking_enabled = true;
static unsigned long last_interrupt_time = 0;
unsigned long interrupt_time = millis();
unsigned long db_btn_press_count = 0;
bool db_toggle_btn_pressed = false;
bool keep_unlocked = false;
bool hass_only_ctrl = false;
bool btn_ctrl_en = true;
bool l_sw_ctrl_en = true; 


#define KEEP_UNLOCKED_ID 20
#define HASS_CTRL_ONLY_ID 21 
#define BTN_CTRL_ENABLED_ID 22
#define L_SW_CTRL_ENABLED_ID 23

MyMessage deadbolt_msg(DEADBOLT_ID, V_LOCK_STATUS);
MyMessage motdet_msg(MOTDET_ID, V_TRIPPED);
MyMessage door_msg(DOOR_ID, V_TRIPPED);
MyMessage keep_unlocked_msg(KEEP_UNLOCKED_ID, V_STATUS);
MyMessage hass_ctrl_only_msg(HASS_CTRL_ONLY_ID, V_STATUS);
MyMessage btn_ctrl_enabled_msg(BTN_CTRL_ENABLED_ID, V_STATUS);
MyMessage l_sw_ctrl_enabled_msg(L_SW_CTRL_ENABLED_ID, V_STATUS);

void before()
{
#ifdef RFID_ENABLED
    // Make sure MFRC is disabled from the SPI bus
    pinMode(RFID_SS_PIN, OUTPUT);
    digitalWrite(RFID_SS_PIN, HIGH);
#endif
}

void presentation()
{
    set_led_strip(MY_OFF);
    // Send the sketch version information to the gateway and Controller
    sendSketchInfo("Backdoor MySensors Node V3", "07-07-22");

    // Register all sensors to gw (they will be created as child devices)
    present(DEADBOLT_ID, S_LOCK);
    present(MOTDET_ID, S_MOTION);
    present(DOOR_ID, S_DOOR);
    present(KEEP_UNLOCKED_ID, S_BINARY);
    present(HASS_CTRL_ONLY_ID, S_BINARY);
    present(BTN_CTRL_ENABLED_ID, S_BINARY);
    present(L_SW_CTRL_ENABLED_ID, S_BINARY);
}

// receive message from Gateway for deadbolt motor
void receive(const MyMessage &msg)
{
    printf("Message received for sensor #%d\n", msg.getSensor());
    if (msg.getType() == V_LOCK_STATUS)
    {
        bool lock_status_recvd = !msg.getBool();  // 1 = unlocked for HASS type lock , so invert it
         printf("Message received for deadbolt!\n");
        // if received LOCK command
        // if (msg.getSensor() == RFID_ID)
        // {
        //   if (!lock_status_recvd) {printf("Access granted has been terminated by HASS!\n");}
        // }
        if (msg.getSensor() == DEADBOLT_ID)
        {
            set_led(LEDS_DB_LAST_TRIG, TRIG_HASS_COLOR);
            if (lock_status_recvd)
            {
                lockDeadbolt();
            }
            else
            {
                unlockDeadbolt();
            }
        }
               
        
    }
    else {
      printf("bool msg recv'd: %d\n", msg.getBool());
      bool bool_recvd = msg.getBool();
      if (msg.getSensor() == KEEP_UNLOCKED_ID) 
            keep_unlocked = bool_recvd;
      else if (msg.getSensor() == HASS_CTRL_ONLY_ID) 
            hass_only_ctrl = bool_recvd;
      else if (msg.getSensor() == BTN_CTRL_ENABLED_ID) 
            btn_ctrl_en = bool_recvd;
      else if (msg.getSensor() == L_SW_CTRL_ENABLED_ID) 
            l_sw_ctrl_en = bool_recvd;
    }
}

void unlockDeadbolt(void)
{
    // if not already unlocked
    //  if (gear_pot_rot > UNLOCKED_POT_POS)
    if (deadbolt_target_state != UNLOCKED || deadbolt_current_state != UNLOCKED)
    {
        // if deadbolt is not currently being unlocked by motor
        if (!motor_on || deadbolt_target_state != UNLOCKED)
        {
            motor_on = true;
            printf("Unlocking Deadbolt...\n");
            digitalWrite(MOTOR_BACKWARD, LOW);
            digitalWrite(MOTOR_FORWARD, HIGH);
            time_motor_started = millis();
            last_time_pot_advanced = millis();
            send(deadbolt_msg.set(deadbolt_target_state ? "0" : "1"));
            set_led(LEDS_MOTOR_STATE, MOTOR_STATE_ON_COLOR);
            set_led(LEDS_DB_T_STATE, DB_STATE_UNLOCKED_COLOR);
            set_led(LEDS_MOTOR_WARN, MOTOR_WARN_NONE_COLOR);
        }
        // make sure that deadbolt target state is updated to proper value
        deadbolt_target_state = UNLOCKED;
        last_time_unlocked = millis();
    }
    // deadbolt is already unlocked
    else
    {
        deadbolt_target_state = UNLOCKED;
        last_time_unlocked = millis();
        printf("unlockDeadbolt(): Deadbolt already unlocked!\n");
        stopDeadbolt();
        //  send(deadbolt_msg.set(deadbolt_target_state?"0":"1"));
    }
    set_led(LEDS_DB_T_STATE, DB_STATE_UNLOCKED_COLOR);
}

void lockDeadbolt(void)
{
    // if deadbolt has not already been locked
    if (deadbolt_target_state != LOCKED || deadbolt_current_state != LOCKED)
    {
        // if deadbolt is not currently being locked by motor
        if (!motor_on || deadbolt_target_state != LOCKED)
        {
          
            motor_on = true;
            printf("Locking Deadbolt...\n");
            digitalWrite(MOTOR_BACKWARD, HIGH);
            digitalWrite(MOTOR_FORWARD, LOW);
            time_motor_started = millis();
            last_time_pot_advanced = millis();
            send(deadbolt_msg.set(deadbolt_target_state ? "0" : "1"));
            set_led(LEDS_MOTOR_STATE, MOTOR_STATE_ON_COLOR);
            set_led(LEDS_DB_T_STATE, DB_STATE_LOCKED_COLOR);
            set_led(LEDS_MOTOR_WARN, MOTOR_WARN_NONE_COLOR);
        }
        // make sure that deadbolt target state is updated to proper value
        deadbolt_target_state = LOCKED;
    }
    // deadbolt is already locked
    else
    {
        deadbolt_target_state = LOCKED;
        //    printf("lockDeadbolt(): Deadbolt already locked!\n");
        stopDeadbolt();
        send(deadbolt_msg.set(deadbolt_target_state ? "0" : "1"));
    }
}

void stopDeadbolt(void)
{
    if (motor_on)
    {
        motor_stalled = false;
        
        motor_on = false;
        printf("Stopping Deadbolt Motor. Action took %dms\n", millis() - time_motor_started);
        digitalWrite(MOTOR_BACKWARD, LOW);
        digitalWrite(MOTOR_FORWARD, LOW);
        
        send(deadbolt_msg.set(deadbolt_target_state ? "0" : "1"));
        set_led(LEDS_MOTOR_STATE, MOTOR_STATE_OFF_COLOR);
        wait(100); // pause to let everything settle to a rest
    }
    set_led(LEDS_DB_C_STATE, deadbolt_current_state == LOCKED ? DB_STATE_LOCKED_COLOR : DB_STATE_UNLOCKED_COLOR);
    gear_pot_val_when_stopped = analogRead(GEAR_POT);
}

void setup()
{

    Serial.begin(115200);
    // put your setup code here, to run once:
    // pinMode(MOT_DET_PIN, INPUT);
    // pinMode(PERSON_PRESENT_PIN, OUTPUT);
    pinMode(MOTOR_FORWARD, OUTPUT);
    pinMode(MOTOR_BACKWARD, OUTPUT);
    pinMode(MOTOR_SLEEP, OUTPUT);
    digitalWrite(DOOR_LIMIT_SWITCH, HIGH);
    digitalWrite(DEADBOLT_TOGGLE_BUTTON, HIGH);
    FastLED.addLeds<NEOPIXEL, LED_STRIP_DATA>(leds, NUM_LEDS);
    FastLED.setBrightness(50);
    attachInterrupt(digitalPinToInterrupt(DOOR_LIMIT_SWITCH), door_limit_switch_irq_handler, CHANGE);
    
//    attachInterrupt(digitalPinToInterrupt(DEADBOLT_TOGGLE_BUTTON), db_toggle_btn_irq_handler, FALLING);
    // enable motor controller
    digitalWrite(MOTOR_SLEEP, HIGH);
    // if (!SETUP_MODE)
    // {
    //   attachInterrupt(digitalPinToInterrupt(MOT_DET_PIN), presenceISR, RISING);
    // }

    //  door_limit_switch_debouncer.attach( DOOR_LIMIT_SWITCH ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
//     deadbolt_toggle_button.attach( DEADBOLT_TOGGLE_BUTTON ,  INPUT_PULLUP ); // USE INTERNAL PULL-UP
    // DEBOUNCE INTERVAL IN MILLISECONDS
    //  door_limit_switch_debouncer.interval(10);
//     deadbolt_toggle_button.interval(10);
    // INDICATE THAT THE HIGH STATE CORRESPONDS TO PHYSICALLY PRESSING THE BUTTON
    //  door_limit_switch_debouncer.setPressedState(HIGH);
//     deadbolt_toggle_button.setPressedState(HIGH);

//    pinMode(ULTRASONIC_TRIG, OUTPUT); // Sets the trigPin as an OUTPUT
//    pinMode(ULTRASONIC_ECHO, INPUT);  // Sets the echoPin as an INPUT

    // mySerial.begin(1200);
    printf("Setup() complete! Serial connected.\n");
}

void calc_hand_distance()
{
    hand_distance = 0;
    for (int i = 0; i < ULTRASONIC_TRIALS; i++)
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

void door_limit_switch_irq_handler()
{
    printf("irq : limit switch %spressed\n", digitalRead(DOOR_LIMIT_SWITCH) == HIGH ? "NOT " : "");

//    if (!hass_only_ctrl && !keep_unlocked && l_sw_ctrl_en)
//    {
      last_interrupt_time = 0;
      interrupt_time = millis();
      // If interrupts come faster than 10ms, assume it's a bounce and ignore
      if (interrupt_time - last_interrupt_time > 10)
      {
          door_limit_switch_pressed = digitalRead(DOOR_LIMIT_SWITCH) == LOW;
      }
      last_interrupt_time = interrupt_time;
//    }
}


void db_toggle_btn_irq_handler()
{
//  printf("db toggle button pressed\n");
//  if (!hass_only_ctrl && btn_ctrl_en)
//  {
//    if (deadbolt_target_state == LOCKED)
//    unlockDeadbolt();
//    else if(deadbolt_current_state == UNLOCKED)
//    lockDeadbolt(); 
//    set_led(LEDS_DB_LAST_TRIG, TRIG_BUTTON_COLOR);
//  }
}


void loop()
{
    if (!main_loop_entered)
    {
        set_led(LEDS_HAND_DET, HAND_NOT_DET_COLOR);
      
        time_motor_started = millis();
        stopDeadbolt();
        door_limit_switch_pressed = digitalRead(DOOR_LIMIT_SWITCH) == LOW; 
        door_open = !door_limit_switch_pressed;
        // update limit switch debouncer
        //    door_limit_switch_debouncer.update();
        //    door_open = door_limit_switch_debouncer.read() != 0;
        // enable motor controller
        digitalWrite(MOTOR_SLEEP, HIGH);
        Serial.print("\n\n\n< ENTERED MAIN LOOP >\n\n\n");
        main_loop_entered = true;
        send(deadbolt_msg.set(deadbolt_current_state ? "0" : "1"));
        send(motdet_msg.set("0"));

        send(door_msg.set(door_open ? "1" : "0"));

        send( keep_unlocked_msg.set( keep_unlocked ? "1" : "0") );
        send( hass_ctrl_only_msg.set( hass_only_ctrl ? "1" : "0") );
        send( btn_ctrl_enabled_msg.set( btn_ctrl_en ? "1" : "0") );
        send( l_sw_ctrl_enabled_msg.set( l_sw_ctrl_en ? "1" : "0") );
        
    }

    //////////////////////////////////////
    // DEADBOLT CONTROL CODE //
    /////////////////////////////////////
    new_gear_pot_rot = (analogRead(GEAR_POT) * 270.0) / 1024.0;
    if ((deadbolt_target_state == LOCKED && new_gear_pot_rot > gear_pot_rot) || (deadbolt_target_state == UNLOCKED && new_gear_pot_rot < gear_pot_rot))
    {
        last_time_pot_advanced = millis();
    }

    gear_pot_rot = new_gear_pot_rot;
    gear_pot_val = analogRead(GEAR_POT);
    //  printf("gear potentiometer rotation [0-270deg]: %d ; raw value: %d\n", gear_pot_rot, gear_pot_val);
      // if deadbolt has been locked
      if (gear_pot_rot >= LOCKED_POT_POS)
      {
          if (deadbolt_current_state != LOCKED)
          {
              if (deadbolt_current_state == UNLOCKED)
              {
                  printf("Deadbolt is now locked\n");
              }

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
              {
                  printf("Deadbolt is now unlocked\n");
                  
              }

              deadbolt_current_state = UNLOCKED;
              stopDeadbolt();
          }
          if (motor_on && deadbolt_target_state == UNLOCKED)
          {
              stopDeadbolt();
          }
      }

      // if motor has stalled
      if (motor_on && (millis() - last_time_pot_advanced >= MOTOR_STALL_TIMEOUT || millis() - time_motor_started > MOTOR_MAX_TIME_ON))
      {
          
          motor_stalled = true;
          set_led(LEDS_MOTOR_WARN, MOTOR_WARN_STALL_COLOR);
//          printf("MOTOR STALLED!\n");
          if (millis() - last_time_pot_advanced >= MOTOR_STALL_TIMEOUT)
          { 
            printf("motor stalled #1\n");
          }
          else if (millis() - time_motor_started > MOTOR_MAX_TIME_ON)
          {
            printf("motor stalled #2\n");
          }
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


      if (door_limit_switch_pressed == door_open)
      {

          // if limit switch is NOT "pressed" ( == 0) then door has been opened
          door_open = !door_limit_switch_pressed;

          // after old wire snapped, using normal closed wire instead now
          //        door_open = door_limit_switch_debouncer.read() == 0;
          
          // send updated door state msg
          send(door_msg.set(door_open ? "1" : "0"));
          set_led(LEDS_DOOR_STATE, door_open ? DOOR_AJAR_COLOR : DOOR_SHUT_COLOR);
          // if door has just been opened
          if (door_open)
          {
              last_time_door_opened = millis();
              // if (last_time_hand_detected > 1000)
              if (!hass_only_ctrl && l_sw_ctrl_en && deadbolt_target_state == LOCKED && deadbolt_current_state != LOCKED)
              {
                  unlockDeadbolt();
                  set_led(LEDS_DB_LAST_TRIG, TRIG_HASS_COLOR);
              }
              printf("Door was just opened!\n");
          }
          // if door has just been closed
          else
          {
              last_time_door_closed = millis();
              printf("Door was just closed!\n");
              if (!hass_only_ctrl && !keep_unlocked && l_sw_ctrl_en)
              {
              lockDeadbolt();
              }
          }
      }

      
  if (digitalRead(DEADBOLT_TOGGLE_BUTTON) == LOW)
  {
    if(db_toggle_btn_pressed != !digitalRead(DEADBOLT_TOGGLE_BUTTON))
    {
      wait(10);
      if (digitalRead(DEADBOLT_TOGGLE_BUTTON) == LOW)
      {
        db_toggle_btn_pressed =  true; // invert it because its pulled HIGH so LOW means pressed
        printf("db toggle button pressed\n");
        if (!hass_only_ctrl && btn_ctrl_en)
        {
          if (deadbolt_target_state == LOCKED)
          unlockDeadbolt();
          else if(deadbolt_current_state == UNLOCKED)
          lockDeadbolt(); 
          set_led(LEDS_DB_LAST_TRIG, TRIG_BUTTON_COLOR);
        }
      }
    }
  }
  
  else if (db_toggle_btn_pressed == true) {
    wait(10);
    if (digitalRead(DEADBOLT_TOGGLE_BUTTON) == HIGH)
    {
      db_toggle_btn_pressed = false;
    }
  }
//
//      
//      deadbolt_toggle_button.update();
//
//      if ( deadbolt_toggle_button.changed() ) {
//          
//          printf("deadbolt button state has changed : %d\n", db_btn_press_count);
//          // if button has been pressed
////          button_pressed = !button_pressed;
//          
//          if (deadbolt_toggle_button.read() == 0 && digitalRead(DEADBOLT_TOGGLE_BUTTON) == LOW && (db_btn_press_count++) % 2 == 0 );
//          {   
//            if (!hass_only_ctrl)
//            {
//              if (deadbolt_target_state == LOCKED)
//              unlockDeadbolt();
//              else if(deadbolt_current_state == UNLOCKED)
//              lockDeadbolt(); 
//              set_led(LEDS_DB_LAST_TRIG, TRIG_BUTTON_COLOR);
//            }
//            printf("deadbolt button has been PRESSED\n");
//          }
//      }

      // activate ultrasonic sensor to read hand distance
      if (ULTRASONIC_ENABLED && millis() - last_time_hand_detected > 500)
      {
          calc_hand_distance();

          // if hand detected
          if (hand_distance < HAND_PRESENT_MAX_DIST)
          {
              set_led(LEDS_HAND_DET, HAND_DET_COLOR);

              last_time_hand_detected = millis();
              // 
              if (millis() - last_time_door_closed > 2000)
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
                      set_led(LEDS_DB_LAST_TRIG, TRIG_HAND_COLOR);
                  }
              }
          }
          else
          {
              set_led(LEDS_HAND_DET, HAND_NOT_DET_COLOR);
          }
      }
      #ifdef LOCAL_AUTO_LOCK_CONTROL

      // if it is time for door to auto re-lock and it is still unlocked and door is shut and deadbolt isnt moving
      if (millis() - last_time_unlocked > KEEP_UNLOCKED_DUR && !door_open && !motor_on && deadbolt_current_state == UNLOCKED)
      {

          printf("Auto re-locking deadbolt!\n");
          lockDeadbolt();
      }

      #endif

    

    //// COMMUNICATE WITH AUTHENTICATION ARDUINO /////

    if (Serial.available() > 0)
    {
        char rx = Serial.read();
        //  bool authenticated = (rx == '1');
        printf("Received %d ('%c') from auth arduino!\n", rx, rx);
        // if rfid or fingerprint authorized
        if (rx == 'r' || rx == 'f')
        {

            printf("Authorized using %s\n", rx == 'r' ? "RFID" : "Fingerprint");
            unlockDeadbolt();
            set_led(LEDS_DB_LAST_TRIG, TRIG_AUTH_COLOR);
            set_led(LEDS_AUTH, (rx == 'r' ? AUTH_RFID_COLOR : AUTH_FINGERPRINT_COLOR));
        }
        // if heartbeat received from 
        else if (rx == 'h')
        {
            set_led(LEDS_AUTH, AUTH_CONNECTED_COLOR);
        }
        else if (rx == 's')
        {
          stopDeadbolt();
        }
    
        else if (rx == 'u')
        {
          unlockDeadbolt();
        }
        else if (rx == 'l')
        {
          lockDeadbolt();
        }
        else if (rx == 'p')
        {
          printf("gear potentiometer rotation [0-270deg]: %d ; raw value: %d\n", gear_pot_rot, gear_pot_val);
        }
    }

    if (!motor_on && millis() - last_time_heartbeat > HEARTBEAT_INTERVAL)
    {
        
        send(deadbolt_msg.set(deadbolt_target_state ? "0" : "1"));
        send(door_msg.set(door_open ? "1" : "0"));
        last_time_heartbeat = millis();
    }

check_to_clear_leds();
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
