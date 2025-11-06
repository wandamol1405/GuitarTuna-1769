// Receptor UART para Arduino UNO
// Recibe por RX (pin 0) y muestra en el monitor serial

void setup() {
  Serial.begin(9600); // Mismo baud rate que la LPC
  Serial.println("Esperando datos de la LPC...");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    Serial.write(c); // Muestra directamente lo que llega
  }
}
