#include <TinyGPS++.h>

TinyGPSPlus gps;

//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#define MAG_X_OFFSET 70
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
  double distance;
  double direction;
  double move_direction;
  
  getData(&latitude,&longnitude,&degree);
  dirDisCalc(latitude,longnitude,34.647375,135.7591138889,&distance,&direction);
  moveDirCalc(direction,degree,&move_direction);
  Serial.print("LAT="); Serial.println(latitude, 10);
  Serial.print("LONG="); Serial.println(longnitude, 10);
  Serial.print("DEG="); Serial.println(degree);
  Serial.print("DIS="); Serial.println(distance,4);
  Serial.print("DIR="); Serial.println(direction);
  Serial.print("MOVE="); Serial.println(move_direction); 
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

  *compass_deg = (float)360 - *compass_deg;
 
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
void dirDisCalc(double d_lat,double d_lng,double g_lat,double g_lng,double *dis,double *dir){
  double pi = 3.14159265359;
  double lat1=d_lat*pi/180;
  double lat2=g_lat*pi/180;
  double lng1=d_lng*pi/180;
  double lng2=g_lng*pi/180;
  double earth_radius=6378.137;
  *dis = earth_radius * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lng2-lng1));
  *dir = 90-(180/pi*(atan2((cos(lat1)*tan(lat2)-sin(lat1)*cos(lng2-lng1)),sin(lng2-lng1))));
  if(*dir<0){
    *dir+=360;
  }
}
void moveDirCalc(double direction,double compass_point,double *move_direction){
  if(direction > compass_point){
    double A = direction-compass_point;
    double B = 360-A;
    if(A>B){
      *move_direction = -B;
    }else{
      *move_direction = A;
    }
  }else{
    double A = direction-compass_point;
    double B = 360-A;
    if(A>B){
      *move_direction = B;
    }else{
      *move_direction = A;
    }
  }
}
