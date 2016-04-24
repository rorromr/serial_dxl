/*
TEST RECEIVE
TX3 -> D
RX3 -> R
.RE,DE +10k pulldown -> HIGH (Enable transmission)


 */


void setup() {
  // initialize both serial ports:
  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(6,OUTPUT);
  digitalWrite(6, HIGH);
}

void loop() {
  // read from port 1, send to port 0:
  Serial3.println("Test send");
  delay(500);
}
