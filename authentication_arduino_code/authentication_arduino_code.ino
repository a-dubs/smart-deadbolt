/*
 * 
 * All the resources for this project: http://randomnerdtutorials.com/
 * Modified by Rui Santos
 * 
 * Created by FILIPEFLOP
 * 
 */


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Arduino Pro Mini Pinout Resrvations ///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

// [ TX   ]   Pin 0  - Serial Comm w/ DB Arduino
// [ RX   ]   Pin 1  - Serial Comm w/ DB Arduino
// [ INT  ]   Pin 2  - 
// [ INT  ] ~ Pin 3  - 
// [      ]   Pin 4  - RFID Auth Accepted LED 
// [      ] ~ Pin 5  - 
// [      ] ~ Pin 6  - 
// [      ]   Pin 7  - KP_C1
// [      ]   Pin 8  - KP_C2
// [      ] ~ Pin 9  - 
// [ SS   ] ~ Pin 10 - RFID Reader SS Signal
// [ MOSI ] ~ Pin 11 - RFID Reader 
// [ MISO ]   Pin 12 - RFID Reader
// [ SCK  ]   Pin 13 - RFID Reader
// [      ]   Pin A0 - 
// [      ]   Pin A1 - 
// [      ]   Pin A2 - KP_C3
// [      ]   Pin A3 - KP_C4
// [      ]   Pin A4 - KP_R1
// [      ]   Pin A5 - KP_R2
// [      ]   Pin A6 - KP_R3
// [      ]   Pin A7 - KP_R4


////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

 
#include <SPI.h>
#include <MFRC522.h>

#define RFID_ACCEPTED_LED_PIN 4
// #define HEARTBEAT_LED_PIN 6
#define RFID_SS_PIN 10
#define RFID_RST_PIN 4
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);   // Create MFRC522 instance.
// SoftwareSerial mySerial(2, 3); // RX, TX


void setup() 
{
  pinMode(RFID_ACCEPTED_LED_PIN, OUTPUT);
  
//   pinMode(HEARTBEAT_LED_PIN, OUTPUT);
//  mySerial.begin(2400);   // Initiate a serial communication
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  // Serial.println("Approximate your card to the reader...");
  // Serial.println();
  delay(1000);
  Serial.begin(115200);
  Serial.write('X');
}

void loop() 
{
  // Look for new cards
  if (  mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) 
  {
    //Show UID on serial monitor
    // Serial.print("UID tag :");
    String content= "";
    byte letter;
    for (byte i = 0; i < mfrc522.uid.size; i++) 
    {
      //  Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      //  Serial.print(mfrc522.uid.uidByte[i], HEX);
       content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
       content.concat(String(mfrc522.uid.uidByte[i], HEX));
    }
    // Serial.println();
    // Serial.print("Message : ");
    content.toUpperCase();
    if (content.substring(1) == "F6 93 AA 13"
  //  || content.substring(1) == ""
    || content.substring(1) == "DE 40 E3 3D"  // coin tag to give to noah
    || content.substring(1) == "04 50 37 CA 60 57 8x0"
    || content.substring(1) == "04 7D 2E CA 60 57 80"
    || content.substring(1) == "04 E5 26 CA 60 57 80"
    || content.substring(1) == "04 23 84 E2 50 60 80")  
  
    {
      // Serial.println("Authorized access");
      // Serial.println();
      Serial.write('r');
      digitalWrite(RFID_ACCEPTED_LED_PIN, HIGH);
      delay(200);
      digitalWrite(RFID_ACCEPTED_LED_PIN, LOW);
    }
   
    else   {
      // Serial.println(" Access denied");
      // delay(3000);
      Serial.write('!');
    }
  }
    // digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    
    Serial.write('h');
  delay(10);
    // digitalWrite(HEARTBEAT_LED_PIN, LOW);
  delay(90);
} 
