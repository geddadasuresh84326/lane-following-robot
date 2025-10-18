int x = 0;
void setup() {
  // put our setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(x);
  x++;
  delay(100);
}
