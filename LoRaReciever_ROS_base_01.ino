
#include <SPI.h>              // include libraries
#include <LoRa.h>
#include <ros.h>
#include <std_msgs/String.h>

const long frequency = 433.5E6;  // LoRa Frequency
const int spreading = 10;
const int csPin = 4;          // LoRa radio chip select
const int resetPin = 2;        // LoRa radio reset
const int irqPin = 3;          // change for your board; must be a hardware interrupt pin

byte localAddress = 0xBB;     // address of this device
byte destination = 0xFF;      // destination to send to
long lastSendTime = 0;        // last send time

const int LED_pin_8 = 8;
const int LED_pin_5 = 5;
// String LED_pin_A3 = "A3";
// String LED_pin_A7 = "A7";

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("received_from_dog", &str_msg);

char inChar[50];

void setup() 
{
  nh.initNode();
  nh.advertise(chatter);

  pinMode(LED_pin_8, OUTPUT);
  pinMode(LED_pin_5, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A7, OUTPUT);
  digitalWrite(LED_pin_8, HIGH);
  digitalWrite(LED_pin_5, HIGH);
  digitalWrite(A3, HIGH);
  digitalWrite(A7, HIGH);
  delay(5000);             // Do not delete this or it may brick the MCU.
  digitalWrite(LED_pin_8, LOW);
  digitalWrite(LED_pin_5, LOW);
  digitalWrite(A3, LOW);
  digitalWrite(A7, LOW);

  // Serial.begin(115200);                   // initialize serial
  // while (!Serial);

  Serial.println("LoRa Reciever");

  // override the default CS, reset, and IRQ pins (optional)
  LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  if (!LoRa.begin(frequency)) 
  {             // initialize ratio at 915 MHz
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
  // parse for a packet, and call onReceive with the result:
  onReceive(LoRa.parsePacket());
  delay(1100);
  Serial.print("inChar: ");Serial.println(inChar);
  str_msg.data = inChar;
  chatter.publish( &str_msg );
  nh.spinOnce();
}

char onReceive(int packetSize) 
{
  if (packetSize == 0) return;          // if there's no packet, return

  // read packet header bytes:
  int recipient = LoRa.read();          // recipient address
  byte sender = LoRa.read();            // sender address
  byte incomingMsgId = LoRa.read();     // incoming msg ID
  byte incomingLength = LoRa.read();    // incoming msg length
  int i = 0;

  
  Serial.println("Message: ");
  while (LoRa.available())
  {
    inChar[i] = (char)LoRa.read();
    i++;
    Serial.print(inChar[i]);
  }

  /*
  if (incomingLength != incoming.length()) 
  {   // check length for error
    Serial.println("error: message length does not match length");
    // return;                             // skip rest of function
  }
  */

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) 
  {
    Serial.println("This message is not for me.");
    // return;                             // skip rest of function
  }

  // if message is for this device, or broadcast, print details:
  Serial.println("Received from: 0x" + String(sender, HEX));
  Serial.println("Sent to: 0x" + String(recipient, HEX));
  Serial.println("Message ID: " + String(incomingMsgId));
  Serial.println("Message length: " + String(incomingLength));
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  Serial.println("Snr: " + String(LoRa.packetSnr()));
  Serial.println();
  digitalWrite(LED_pin_8, HIGH);
  delay(50);
  digitalWrite(LED_pin_8, LOW);

  return inChar;
}
