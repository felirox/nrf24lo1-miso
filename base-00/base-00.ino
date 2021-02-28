/*#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

RF24 radio(10, 9);               // nRF24L01 (CE,CSN)
RF24Network network(radio);      // Include the radio in the network
const uint16_t this_node = 00;   // Address of this node in Octal format ( 04,031, etc)

void setup() {
  // put your setup code here, to run once:
  SPI.begin();
  radio.begin();
  network.begin(90, this_node);  //(channel, node address)
  Serial.begin(115200);

}

void loop() {
 network.update();
  //===== Receiving =====//
  while ( network.available() ) {     // Is there any incoming data?
    RF24NetworkHeader header;
    String incomingData;
    network.read(header, &incomingData, sizeof(incomingData)); // Read the incoming data
    Serial.println(incomingData);
  }
  delay(10);

}



*/

//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
const byte address1[6] = "00001";
const byte address2[6] = "00002";
void(* resetFunc) (void) = 0;
void setup()
{
  while (!Serial);
    Serial.begin(9600);
  
  radio.begin();
  
  //set the address
 // radio.openReadingPipe(0, address);
  
  //Set module as receiver
  //radio.startListening();
}

void loop()
{
  radio.openReadingPipe(0, address1);
  
  //Set module as receiver
  radio.startListening();
  //Read the data if available in buffer
  if (radio.available())
  {
    char text[320] = {0};
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }
 delay(100);
  //

radio.stopListening();
delay(10);

 radio.openReadingPipe(0, address2); 
 radio.startListening();
  //Set module as receiver
  //Read the data if available in buffer
  if (radio.available())
  {
    char text[320] = {0};
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }

  delay(100);
radio.stopListening();

}
