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
  double latitude,longnitude,degree;
  getData(&latitude,&longnitude,&degree);
  Serial.print("LAT="); Serial.println(latitude, 10);
  Serial.print("LONG="); Serial.println(longnitude, 10);
  Serial.print("ALT="); Serial.println(gps.altitude.meters());
}

void getData(double *gps_lat, double *gps_lng, double *compass_deg) {
  
  
  while (1) {
    while (Serial1.available() > 0) {
      char c = Serial1.read();
//       Serial.print(c);
      gps.encode(c);
      if (gps.location.isUpdated()) {
        *gps_lat = gps.location.lat();
        *gps_lng = gps.location.lng();
        return ;
      }
    }
  }

}
