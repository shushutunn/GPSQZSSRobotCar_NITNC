#ifndef GPS_QZSS_NITNC_h
#define GPS_QZSS_NITNC_h
#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <TinyGPS++.h>

#define MAG_X_OFFSET 45
#define MAG_Y_OFFSET 13

#define  L_MOTOR_INA 12
#define  L_MOTOR_INB 10
#define  L_MOTOR_PWM 11
#define  R_MOTOR_INA 4
#define  R_MOTOR_INB 2
#define  R_MOTOR_PWM 3


class GPSQZSSRobotCarNITNC{
  public:
  	GPSQZSSRobotCarNITNC();
	void getData(double *gps_lat, double *gps_lng, float *compass_deg);
	void setMotorPower(int l_power,int r_power,boolean l_break,boolean r_break);	
  private:
    uint8_t l_motor_state;
    uint8_t r_motor_state;
};

#endif
