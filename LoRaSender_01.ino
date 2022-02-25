#include <SPI.h>
#include <LoRa.h>

int counter = 0;
const long frequency = 433E6;  // LoRa Frequency
const int csPin = 4;          // LoRa radio chip select
const int resetPin = 2;        // LoRa radio reset
const int irqPin = 3;          // change for your board; must be a hardware interrupt pin
int LED_pin_8 = 8;

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends


void setup() 
{
  pinMode(LED_pin_8, OUTPUT);
  digitalWrite(LED_pin_8, HIGH);
  delay(1000);
  digitalWrite(LED_pin_8, LOW);
  Serial.begin(9600);
  // while (!Serial);

  Serial.println("LoRa Sender non-blocking");
    // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin


  if (!LoRa.begin(433E6)) 
  {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSpreadingFactor(12);           // ranges from 6-12,default 7 see API docs
  Serial.println("LoRa init succeeded.");
}

void loop() 
{
  // wait until the radio is ready to send a packet
  while (LoRa.beginPacket() == 0) 
  {
    Serial.print("waiting for radio ... ");
    digitalWrite(LED_pin_8, HIGH);
    delay(50);
    digitalWrite(LED_pin_8, LOW);
    delay(50);
  }

  Serial.print("Sending packet non-blocking: ");
  Serial.println(counter);

  // send in async / non-blocking mode
  String outgoing = "Hello Asshole!";
  LoRa.beginPacket();
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);
  // LoRa.print(counter);
  LoRa.endPacket(true); // true = async / non-blocking mode

  counter++;
  delay(1000);
}
