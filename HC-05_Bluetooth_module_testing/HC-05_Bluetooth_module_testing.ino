void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400); //ensure serial monitor is also set to 38400 baud
  //the default setting on HC-05 is typically at 9600 baud/second but mine was by default at 38400
  }

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  static int count; //'static' just means the value of count is remember/preserved in the next cycle of void loop(). Its starting value is 0.
  count++;
  Serial.print("HC-05 says 'Hello world'. Number of times: ");
  Serial.println(count);
}
