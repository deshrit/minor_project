unsigned long elapsed_time, current_time;
int x = 0;

void setup() {
  Serial.begin(9600);
  elapsed_time = millis();
}

void loop() {
  delay(1000);
 current_time = millis();
 Serial.print("\nElpased time: ");
 Serial.print(elapsed_time);
 Serial.print("\tCurrent time: ");
 Serial.print(current_time);
}
