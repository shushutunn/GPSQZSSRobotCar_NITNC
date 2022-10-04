#include <TinyGPS++.h>

TinyGPSPlus gps;

//TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#define MAG_X_OFFSET 70
#define MAG_Y_OFFSET 13
#define CENTER_SRC 0.018
#define MAIN_SRC 0.004
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
#define  L_MOTOR_INA 12
#define  L_MOTOR_INB 10
#define  L_MOTOR_PWM 11
#define  R_MOTOR_INA 4
#define  R_MOTOR_INB 2
#define  R_MOTOR_PWM 3
#define  BTN_1 9

double distance;
double direction;
double latitude, longnitude;
float degree;
double move_direction;
void setup() {
  pinMode(L_MOTOR_INA, OUTPUT);
  pinMode(L_MOTOR_INB, OUTPUT);
  pinMode(R_MOTOR_INA, OUTPUT);
  pinMode(R_MOTOR_INB, OUTPUT);
  pinMode(BTN_1, INPUT_PULLUP);

  Serial.begin(9600);

  Serial.println("Goodnight moon!");

  Serial1.begin(38400);
  Serial1.println("Hello, world?");
  while (digitalRead(BTN_1) == HIGH);

  if (!mag.begin())
  {
    //There was a problem detecting the LIS2MDL ... check your connections
    Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
    while (1);
  }

  //app_pi(34.64775033, 135.759442400);
  //lotate_pi(34.64775033, 135.759442400,0);
}

void loop() {
  double pA_lat=34.64774950;
  double pA_lng=135.75943433;
  double pB_lat=34.64763883;
  double pB_lng=135.75926217;
getData(&latitude,&longnitude,&degree);
  Serial.print(gps.time.hour()+9);Serial.print(":");Serial.print(gps.time.minute());Serial.print(":");Serial.println(gps.time.second());
//  app_pi(pA_lat,pA_lng);
//  lotate_pi(pA_lat,pA_lng,pB_lat,pB_lng,0);
//  app_pi(pB_lat,pB_lng);
//  lotate_pi(pB_lat,pB_lng,pA_lat,pA_lng,1);
//  
//  
/*
  getData(&latitude, &longnitude, &degree);
  dirDisCalc(latitude, longnitude, 34.64775033, 135.759442400, &distance, &direction);
  moveDirCalc(direction, degree, &move_direction);
  
    Serial.print("LAT="); Serial.println(latitude, 10);
    Serial.print("LONG="); Serial.println(longnitude, 10);
    Serial.print("ALT="); Serial.println(degree);
    Serial.print("DIS="); Serial.println(distance,4);
    Serial.print("DIR="); Serial.println(direction);
    Serial.print("MOVE="); Serial.println(move_direction);

  */

  //
}
void setMotorPower(int l_power, int r_power, boolean m_break) {

  if (m_break == true) {
    digitalWrite(L_MOTOR_INA, HIGH);
    digitalWrite(L_MOTOR_INB, HIGH);
    analogWrite(L_MOTOR_PWM, 0);
    digitalWrite(R_MOTOR_INA, HIGH);
    digitalWrite(R_MOTOR_INB, HIGH);
    analogWrite(R_MOTOR_PWM, 0);
    return;
  }

  if (l_power > 0) {
    digitalWrite(L_MOTOR_INA, HIGH);
    digitalWrite(L_MOTOR_INB, LOW);
  } else if (l_power < 0) {
    digitalWrite(L_MOTOR_INA, LOW);
    digitalWrite(L_MOTOR_INB, HIGH);
  } else {
    digitalWrite(L_MOTOR_INA, LOW);
    digitalWrite(L_MOTOR_INB, LOW);
  }
  analogWrite(L_MOTOR_PWM, abs(l_power));

  if (r_power > 0) {
    digitalWrite(R_MOTOR_INA, HIGH);
    digitalWrite(R_MOTOR_INB, LOW);
  } else if (r_power < 0) {
    digitalWrite(R_MOTOR_INA, LOW);
    digitalWrite(R_MOTOR_INB, HIGH);
  } else {
    digitalWrite(R_MOTOR_INA, LOW);
    digitalWrite(R_MOTOR_INB, LOW);
  }
  analogWrite(R_MOTOR_PWM, abs(r_power));
}
void getData(double *gps_lat, double *gps_lng, float *compass_deg) {

  sensors_event_t event;
  mag.getEvent(&event);
  float Pi = 3.14159;
  *compass_deg = (atan2(event.magnetic.y + MAG_Y_OFFSET, event.magnetic.x + MAG_X_OFFSET) * 180) / Pi;
  if (*compass_deg < 0)
  {
    *compass_deg = 360 + *compass_deg;
  }

  *compass_deg = 360 - *compass_deg;

  unsigned long gps_start_time = millis();
  while (1) {
    while (Serial1.available() > 0) {
      char c = Serial1.read();
      //       Serial.print(c);
      gps.encode(c);
      if (gps.location.isUpdated() || millis() > gps_start_time + 50) {
        *gps_lat = gps.location.lat();
        *gps_lng = gps.location.lng();
        return ;
      }
    }
  }

}
void dirDisCalc(double d_lat, double d_lng, double g_lat, double g_lng, double *dis, double *dir) {
  double pi = 3.14159265359;
  double lat1 = d_lat * pi / 180;
  double lat2 = g_lat * pi / 180;
  double lng1 = d_lng * pi / 180;
  double lng2 = g_lng * pi / 180;
  double earth_radius = 6378.137;
  *dis = earth_radius * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lng2 - lng1));
  *dir = 90 - (180 / pi * (atan2((cos(lat1) * tan(lat2) - sin(lat1) * cos(lng2 - lng1)), sin(lng2 - lng1))));
  if (*dir < 0) {
    *dir += 360;
  }

}
double disCalc(double d_lat, double d_lng, double g_lat, double g_lng) {
  double pi = 3.14159265359;
  double lat1 = d_lat * pi / 180;
  double lat2 = g_lat * pi / 180;
  double lng1 = d_lng * pi / 180;
  double lng2 = g_lng * pi / 180;
  double earth_radius = 6378.137;
  double dis = earth_radius * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lng2 - lng1));
  return dis;
}
void moveDirCalc(double direction, double compass_point, double *move_direction) {
  if (direction > compass_point) {
    double A = direction - compass_point;
    double B = 360 - A;
    if (A > B) {
      *move_direction = -B;
    } else {
      *move_direction = A;
    }
  } else {
    double A = direction - compass_point;
    double B = 360 - A;
    if (A > B) {
      *move_direction = B;
    } else {
      *move_direction = A;
    }
  }
}
void app_pi(double g_lat, double g_lng) {
  while(1){
    
  getData(&latitude,&longnitude,&degree);
  dirDisCalc(latitude, longnitude, g_lat, g_lng, &distance, &direction);
  moveDirCalc(direction, degree, &move_direction);
  
  if (move_direction < 10 && move_direction > -10) {
    if(distance < 0.005){
      setMotorPower(70,70,false);
    }else{
      setMotorPower(150, 150, false);
    }
  } else if (move_direction < 45 && move_direction > 0) {
    setMotorPower(200, 0, false);
  } else if (move_direction > -45 && move_direction < 0) {
    setMotorPower(0, 200, false);
  } else if (move_direction < 0) {
    setMotorPower(-200, 200, false);
  } else {
    setMotorPower(200, -200, false);
  }
  
  Serial.print("app:lat=");Serial.println(latitude,10);
  Serial.print("app:lng=");Serial.println(longnitude,10);
  Serial.print("app:distance=");Serial.println(distance,10);
      Serial.print("app:move_direction=");Serial.println(move_direction,10);
    if(distance<0.002){
      setMotorPower(0,0,true);
      break;
    }
  }
}

  void lotate_pi(double p_lat,double p_lng,double next_p_lat,double next_p_lng,int clock){
    double other_dist;
    double tmp;
    bool over=false;
    while(1){
            getData(&latitude,&longnitude,&degree);
  dirDisCalc(latitude,longnitude,p_lat,p_lng,&distance,&direction);
  moveDirCalc(direction,degree,&move_direction);
      Serial.print("lotate:distance=");Serial.println(distance,10);
  //distance degree

    if(clock==0){
      if(distance>0.0015){
        move_direction-=50;
      }else if(distance<0.0005){
        move_direction-=130;
      }else{
        move_direction-=90;
      }
     }else{
      if(distance>0.0015){
        move_direction+=50;
      }else if(distance<0.0005){
        move_direction+=130;
      }else{
        move_direction+=90;
      }
     }
     if(move_direction<-180){
        move_direction+=360;
      }else if(move_direction>180){
        move_direction-=360;
      }
    Serial.print("lotate:move_direction=");Serial.println(move_direction,10); 
      if(move_direction<20&&move_direction>-20){
        setMotorPower(100,100,false);
      }else if(move_direction<0){
        setMotorPower(-200,200,false);
      }else{
        setMotorPower(250,-100,false);
      }
      
      dirDisCalc(latitude,longnitude,next_p_lat,next_p_lng,&other_dist,&tmp);
      if(over==true&&other_dist<0.0195){
        break;
        }else if(other_dist>0.0205){
          over = true;
        }
      }
    
  }

/*
  void move_ctr(double[2] r,double[2] l,double[2] c,double *lat,double *lng){
  r_m_dis=disCalc(r[1],r[0],*lat,*lng);
  l_m_dis=disCalc(l[1],l[0],*lat,*lng);
  c_m_dis=disCalc(c[1],c[0],*lat,*lng);
  moveDirCalc(direction,degree,&move_direction)
  if(r_m_dis<MAIN_SRC&&c_m_dis<CENTER_SRC){
    // to center set l
  }else if(l_m_dis<MAIN_SRC&&c_m_dis<CENTER_SRC){
    // to center set r
  }else if(r_m_dis<MAIN_SRC){

     move_direction-=50
     motor_p(&move_direction)
     //motor_power_set
    // keep 90 rad.right lotate
  }else if(l_m_dis<MAIN_SRC){
     move_direction+=50
     motor_p(&move_direction)
    // keep 90 rad.left lotate
  }
  }7
*/
