#include <EEPROM.h>
#include <SoftwareSerial.h>

void setup() {
  // put your setup code here, to run once:
EEPROM.begin(512);
int i =0;
pinMode(LED_BUILTIN, OUTPUT);
digitalWrite(LED_BUILTIN, LOW);
Serial.begin(9600);
Serial.println("Setting Up EEPROM");

  for(i=0; i<512; i++){
    EEPROM.write(i,1);
  }
  EEPROM.write(0x80, 4);
  EEPROM.commit();
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Done");
}

void loop() {
  // put your main code here, to run repeatedly:

}
