/*
 * 
 * All the resources for this project: http://randomnerdtutorials.com/
 * Modified by Rui Santos
 * 
 * Created by FILIPEFLOP
 * 
 */
 
#include <SPI.h>
#include <MFRC522.h>

#define LED_PIN 5
#define RFID_SS_PIN 10
#define RFID_RST_PIN 4
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);   // Create MFRC522 instance.
// SoftwareSerial mySerial(2, 3); // RX, TX
void setup() 
{
  pinMode(LED_PIN, OUTPUT);
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
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }
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
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
  }
 
  else   {
    // Serial.println(" Access denied");
    // delay(3000);
    Serial.write('x');
  }
} 
