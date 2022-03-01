/*
  LoRa Duplex communication

  Sends a message every half second, and polls continually
  for new incoming messages. Implements a one-byte addressing scheme,
  with 0xFF as the broadcast address.

  Uses readString() from Stream class to read payload. The Stream class'
  timeout may affect other functuons, like the radio's callback. For an

  created 28 April 2017
  by Tom Igoe
*/
#include <SPI.h>              // include libraries
#include <LoRa.h>

const long frequency = 433.5E6;  // LoRa Frequency
int spreading = 10;
const int csPin = 4;          // LoRa radio chip select
const int resetPin = 2;        // LoRa radio reset
const int irqPin = 3;          // change for your board; must be a hardware interrupt pin

String outgoing;              // outgoing message

byte msgCount = 0;            // count of outgoing messages
byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time
int interval = 2000;          // interval between sends

int LED_pin_8 = 8;
int LED_pin_5 = 5;
String LED_pin_A3 = "A3";
String LED_pin_A7 = "A7";

void setup() 
{
  pinMode(LED_pin_8, OUTPUT);
  pinMode(LED_pin_5, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A7, OUTPUT);
  digitalWrite(LED_pin_8, HIGH);
  digitalWrite(LED_pin_5, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A7, HIGH);
  delay(1000);
  digitalWrite(LED_pin_8, LOW);
  digitalWrite(LED_pin_5, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A7, LOW);
  Serial.begin(9600);                   // initialize serial
  while (!Serial);

  Serial.println("LoRa Reciever");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(frequency)) {             // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                       // if failed, do nothing
  }
  // LoRa.setSpreadingFactor(12);           // ranges from 6-12,default 7 see API docs
  LoRa.setSpreadingFactor(spreading);           // ranges from 6-12,default 7 see API docs
  // LoRa.setSignalBandwidth(31.25E3); 
  // LoRa.setCodingRate4(5);
  // LoRa.enableCrc();
  // LoRa.writeRegister(REG_PA_CONFIG, PA_BOOST | (17 - 2));
  // LoRa.writeRegister(REG_PA_DAC, 0x87);  //turn on 3rd amplifier, see 5.4.3
  // LoRa.writeRegister(REG_OCP, 0x20 | 18); //increace power protection to 150mA RegOcp, see 5.4.4
  Serial.println("LoRa init succeeded.");
}

void loop() 
{
/*
  if (millis() - lastSendTime > interval) 
  {
    String message = "HeLoRa CatShite World!";   // send a message
    sendMessage(message);
    Serial.println("Sending " + message);
    lastSendTime = millis();            // timestamp the message
    interval = random(2000) + 1000;    // 2-3 seconds
  }
*/
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
}

void sendMessage(String outgoing) {
  LoRa.beginPacket();                   // start packet
  LoRa.write(destination);              // add destination address
  LoRa.write(localAddress);             // add sender address
  LoRa.write(msgCount);                 // add message ID
  LoRa.write(outgoing.length());        // add payload length
  LoRa.print(outgoing);                 // add payload
  LoRa.endPacket();                     // finish packet and send it
  msgCount++;                           // increment message ID
}

void onReceive(int packetSize) 
{
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length

  String incoming = "";

  while (LoRa.available()) 
  {
    incoming += (char)LoRa.read();
  }

  if (incomingLength != incoming.length()) 
  {   // check length for error
    Serial.println("error: message length does not match length");
    return;                             // skip rest of function
  }

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) 
  {
    Serial.println("This message is not for me.");
    return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("Message: " + incoming);
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  digitalWrite(LED_pin_8, HIGH);
  delay(50);
  digitalWrite(LED_pin_8, LOW);
}
