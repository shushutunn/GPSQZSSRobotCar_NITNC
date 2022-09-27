#include"GPS_QZSS_NITNC.h"
GPSQZSSRobotCarNITNC Robot;

void setup() {

}


void loop() {
  double latitude,longnitude;
  float degree;
  Robot.getData(&latitude,&longnitude,&degree);
  Serial.print("LAT="); Serial.println(latitude, 10);
  Serial.print("LONG="); Serial.println(longnitude, 10);
  Serial.print("ALT="); Serial.println(degree);
  Robot.setMotorPower(500,500,false,false);
}
}
