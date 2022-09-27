#include <TinyGPS++.h>

TinyGPSPlus gps;

//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#define MAG_X_OFFSET 45
#define MAG_Y_OFFSET 13

Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

void setup() {
  
  Serial.begin(9600);

  Serial.println("Goodnight moon!");

  Serial1.begin(38400);
  Serial1.println("Hello, world?");

  if(!mag.begin())
  {
    /* There was a problem detecting the LIS2MDL ... check your connections */
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    while(1);
  }
}

void loop() {
  double latitude,longnitude;
  float degree;
  getData(&latitude,&longnitude,&degree);
  Serial.print("LAT="); Serial.println(latitude, 10);
  Serial.print("LONG="); Serial.println(longnitude, 10);
  Serial.print("ALT="); Serial.println(degree);
}

void getData(double *gps_lat, double *gps_lng, float *compass_deg) {

  sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  *compass_deg = (atan2(event.magnetic.y+MAG_Y_OFFSET,event.magnetic.x+MAG_X_OFFSET) * 180) / Pi;
  if (*compass_deg < 0)
  {
    *compass_deg = 360 + *compass_deg;
  }
 
  unsigned long gps_start_time = millis();
  while (gps_start_time - millis() < 500) {
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
