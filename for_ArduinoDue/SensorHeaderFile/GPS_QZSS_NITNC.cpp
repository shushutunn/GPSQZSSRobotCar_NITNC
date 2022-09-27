#include "Arduino.h"
#include "GPS_QZSS_NITNC.h"

GPSQZSSRobotCarNITNC::GPSQZSSRobotCarNITNC(){
	TinyGPSPlus gps;
	Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

	pinMode(L_MOTOR_INA,OUTPUT);	
	pinMode(L_MOTOR_INB,OUTPUT);
	pinMode(R_MOTOR_INA,OUTPUT);
	pinMode(R_MOTOR_INB,OUTPUT);

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

void GPSQZSSRobotCarNITNC::getData(double *gps_lat, double *gps_lng, float *compass_deg) {

  sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  *compass_deg = (atan2(event.magnetic.y+MAG_Y_OFFSET,event.magnetic.x+MAG_X_OFFSET) * 180) / Pi;
  if (*compass_deg < 0)
  {
    *compass_deg = 360 + *compass_deg;
  }
 
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

void GPSQZSSRobotCarNITNC::	setMotorPower(int l_power,int r_power,boolean l_break,boolean r_break){

  if(l_break == true || r_break == true){
  if(l_break == true){
    digitalWrite(L_MOTOR_INA,HIGH);
    digitalWrite(L_MOTOR_INB,HIGH);
    analogWrite(L_MOTOR_PWM,0);
    
  }
  if(r_break == true){
    digitalWrite(R_MOTOR_INA,HIGH);
    digitalWrite(R_MOTOR_INB,HIGH);
    analogWrite(R_MOTOR_PWM,0);
    
  }
  return;
  }
  
  if(l_power > 0){
      digitalWrite(L_MOTOR_INA,HIGH);
      digitalWrite(L_MOTOR_INB,LOW);
    }else if(l_power < 0){
      digitalWrite(L_MOTOR_INA,LOW);
      digitalWrite(L_MOTOR_INB,HIGH);
    }else{
      digitalWrite(L_MOTOR_INA,LOW);
      digitalWrite(L_MOTOR_INB,LOW);
    }
   analogWrite(L_MOTOR_PWM,abs(l_power));

    if(r_power > 0){
      digitalWrite(R_MOTOR_INA,HIGH);
      digitalWrite(R_MOTOR_INB,LOW);
    }else if(r_power < 0){
      digitalWrite(R_MOTOR_INA,LOW);
      digitalWrite(R_MOTOR_INB,HIGH);
    }else{
      digitalWrite(R_MOTOR_INA,LOW);
      digitalWrite(R_MOTOR_INB,LOW);
    }
   analogWrite(R_MOTOR_PWM,abs(r_power));
}
