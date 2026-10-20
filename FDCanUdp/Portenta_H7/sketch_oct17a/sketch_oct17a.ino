void setup() {
  // Initialize the RGB LED pins as outputs
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
}

void blink(int led, unsigned long msecs = 1000) {
  digitalWrite(led, LOW);   // LOW turns the Portenta LED ON
  delay(msecs);
  digitalWrite(led, HIGH);  // HIGH turns the Portenta LED OFF
  delay(msecs);
}

void loop() {
  blink(LEDR); // Blink Red
  blink(LEDG); // Blink Green
  blink(LEDB); // Blink Blue
}
