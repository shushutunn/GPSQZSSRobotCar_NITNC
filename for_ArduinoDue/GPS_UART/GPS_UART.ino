#include <TinyGPS++.h>
 
TinyGPSPlus gps;

//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);
 
void setup() {  
 // Open serial communications and wait for port to open:
 Serial.begin(9600);
 while (!Serial) {
 ; // wait for serial port to connect. Needed for native USB port only
 }
 
 Serial.println("Goodnight moon!");
 
 // set the data rate for the SoftwareSerial port
 Serial1.begin(38400);
 Serial1.println("Hello, world?");
}
 
void loop() { // run over and over
 while (Serial1.available() > 0){
 char c = Serial1.read();
// Serial.print(c);
 gps.encode(c);
 if (gps.location.isUpdated()){
 Serial.print("LAT="); Serial.println(gps.location.lat(), 10);
 Serial.print("LONG="); Serial.println(gps.location.lng(), 10);
 Serial.print("ALT="); Serial.println(gps.altitude.meters());
 }
 }
}
