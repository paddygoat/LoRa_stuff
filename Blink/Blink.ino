


void setup() 
{
  pinMode(5, OUTPUT);
  // pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(8, OUTPUT);
}

// the loop function runs over and over again forever
void loop() 
{
  digitalWrite(5, HIGH);
  delay(200);
  digitalWrite(5, LOW);

  // digitalWrite(16, HIGH);
  // delay(200);
  // digitalWrite(16, LOW);

  digitalWrite(17, HIGH);
  delay(200);
  digitalWrite(17, LOW);

  digitalWrite(8, HIGH);
  delay(200);
  digitalWrite(8, LOW);  
}
