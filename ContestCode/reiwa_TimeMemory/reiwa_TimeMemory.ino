#include <TinyGPS++.h>

TinyGPSPlus gps;

// TinyGPSCustom magneticVariation(gps, "GPRMC", 10);

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#define MAG_X_OFFSET 70
#define MAG_Y_OFFSET 13
#define CENTER_SRC 0.018
#define MAIN_SRC 0.004
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);
#define L_MOTOR_INA 12
#define L_MOTOR_INB 10
#define L_MOTOR_PWM 11
#define R_MOTOR_INA 4
#define R_MOTOR_INB 2
#define R_MOTOR_PWM 3
#define BTN_1 9
#define BTN_2 8
#define B_LED 7
#define G_LED 6
#define R_LED 5

#define BUZ A0

#define COUNTER_ROTATION_SPEED 180
#define ONE_SIDE_ROTATION_SPEED 255
#define FAST_PROG_SPEED 200
#define SLOW_PROG_SPEED 70

#define disk1 0x50 // Address of 24LC256 eeprom chip

double distance;
double direction;
double latitude, longnitude;
float degree;
double move_direction;
int limittime;
int recordtime;

void setup()
{
	pinMode(L_MOTOR_INA, OUTPUT);
	pinMode(L_MOTOR_INB, OUTPUT);
	pinMode(R_MOTOR_INA, OUTPUT);
	pinMode(R_MOTOR_INB, OUTPUT);
	pinMode(BTN_1, INPUT_PULLUP);
	pinMode(BTN_2, INPUT_PULLUP);
	pinMode(B_LED, OUTPUT);
	pinMode(R_LED, OUTPUT);
	pinMode(G_LED, OUTPUT);
	pinMode(BUZ, OUTPUT);

	digitalWrite(B_LED, LOW);
	digitalWrite(R_LED, HIGH);
	digitalWrite(G_LED, LOW);
	digitalWrite(BUZ, LOW);

	Serial.begin(9600);

	Serial.println("Goodnight moon!");

	Serial1.begin(38400);
	Serial1.println("Hello, world?");
	double tmp;
	double tmpp;
	float tmppp;

	getDataFirst(&tmp, &tmpp, &tmppp);
	if (!mag.begin())
	{
		// There was a problem detecting the LIS2MDL ... check your connections
		Serial.println("Ooops, no LIS2MDL detected ... Check your wiring!");
		while (1)
			;
	}

	digitalWrite(R_LED, LOW);
	while (digitalRead(BTN_1) == HIGH)
	{
		digitalWrite(B_LED, HIGH);
		getData(&tmp, &tmpp, &tmppp);
		if (digitalRead(BTN_2) == LOW)
		{
			writeEEPROM(disk1, 0, (byte)gps.time.hour());
			writeEEPROM(disk1, 1, (byte)gps.time.minute());
			writeEEPROM(disk1, 2, (byte)gps.time.second());
			digitalWrite(G_LED, HIGH);
			delay(200);
			digitalWrite(G_LED, LOW);
		}
	}
	digitalWrite(B_LED, LOW);
	digitalWrite(G_LED, HIGH);

	// Serial.print(readEEPROM(disk1, 0));
	// Serial.print("\t");
	// Serial.print(readEEPROM(disk1, 1));
	// Serial.print("\t");
	// Serial.println(readEEPROM(disk1, 2));

	limittime = (readEEPROM(disk1, 0) + 9) * 3600 + (readEEPROM(disk1, 1) + 3) * 60 + readEEPROM(disk1, 2);
	Serial.print("First");

	// Serial.print(int((gps.time.hour()+9)*3600+(gps.time.minute())*60+gps.time.second()));
	// Serial.print("\t");
	// Serial.println(limittime);
}

void loop()
{
	// 35.66797048 139.79384333

	double pA_lat = 35.66797048;
	double pA_lng = 139.79384333;
	double pB_lat = 35.66809849;
	double pB_lng = 139.79399841;
	double center_lat = 35.66803451;
	double center_lng = 139.79392100;
	//時間の取得サンプル。グリニッジ標準時で出てくるので9時間足す。getData実行後でないと取得できない。
	// Serial.print(gps.time.hour()+9);Serial.print(":");Serial.print(gps.time.minute());Serial.print(":");Serial.println(gps.time.second());

	// TimeKeeping Test
	//  while(1){
	//   getData(&latitude, &longnitude, &degree);

	// 	// Serial.print(int((gps.time.hour()+9)*3600+(gps.time.minute())*60+gps.time.second()));
	// 	// Serial.print("\t");
	// 	// Serial.println(limittime);
	// if(limittime-int((gps.time.hour()+9)*3600+(gps.time.minute())*60+gps.time.second())<160	){
	//     //中心地に向かう
	//     app_center(center_lat,center_lng);
	// 	// Serial.print("MOVE");
	//     while(1){

	//       }
	//     }
	// }
	// while(1);

	app_pi(pA_lat, pA_lng);
	lotate_pi(pA_lat, pA_lng, pB_lat, pB_lng, 0);
	if (limittime - int((gps.time.hour() + 9) * 3600 + (gps.time.minute()) * 60 + gps.time.second()) < 60)
	{
		//中心地に向かう
		app_center(center_lat, center_lng);

		while (1)
		{
		}
	}
	app_pi(pB_lat, pB_lng);
	lotate_pi(pB_lat, pB_lng, pA_lat, pA_lng, 1);
	if (limittime - int((gps.time.hour() + 9) * 3600 + (gps.time.minute()) * 60 + gps.time.second()) < 60)
	{
		//中心地に向かう
		app_center(center_lat, center_lng);

		while (1)
		{
		}
	}
}
void setMotorPower(int l_power, int r_power, boolean m_break)
{

	if (m_break == true)
	{
		digitalWrite(L_MOTOR_INA, HIGH);
		digitalWrite(L_MOTOR_INB, HIGH);
		analogWrite(L_MOTOR_PWM, 0);
		digitalWrite(R_MOTOR_INA, HIGH);
		digitalWrite(R_MOTOR_INB, HIGH);
		analogWrite(R_MOTOR_PWM, 0);
		return;
	}

	if (l_power > 0)
	{
		digitalWrite(L_MOTOR_INA, HIGH);
		digitalWrite(L_MOTOR_INB, LOW);
	}
	else if (l_power < 0)
	{
		digitalWrite(L_MOTOR_INA, LOW);
		digitalWrite(L_MOTOR_INB, HIGH);
	}
	else
	{
		digitalWrite(L_MOTOR_INA, LOW);
		digitalWrite(L_MOTOR_INB, LOW);
	}
	analogWrite(L_MOTOR_PWM, abs(l_power));

	if (r_power > 0)
	{
		digitalWrite(R_MOTOR_INA, HIGH);
		digitalWrite(R_MOTOR_INB, LOW);
	}
	else if (r_power < 0)
	{
		digitalWrite(R_MOTOR_INA, LOW);
		digitalWrite(R_MOTOR_INB, HIGH);
	}
	else
	{
		digitalWrite(R_MOTOR_INA, LOW);
		digitalWrite(R_MOTOR_INB, LOW);
	}
	analogWrite(R_MOTOR_PWM, abs(r_power));
}
void getData(double *gps_lat, double *gps_lng, float *compass_deg)
{

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
	while (1)
	{
		while (Serial1.available() > 0)
		{
			char c = Serial1.read();
			//       Serial.print(c);
			gps.encode(c);
			if (gps.location.isUpdated() || millis() > gps_start_time + 20)
			{
				*gps_lat = gps.location.lat();
				*gps_lng = gps.location.lng();
				return;
			}
		}
	}
}
void getDataFirst(double *gps_lat, double *gps_lng, float *compass_deg)
{

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
	while (1)
	{
		while (Serial1.available() > 0)
		{
			char c = Serial1.read();
			//       Serial.print(c);
			gps.encode(c);
			if (gps.location.isUpdated())
			{
				*gps_lat = gps.location.lat();
				*gps_lng = gps.location.lng();
				return;
			}
		}
	}
}
void dirDisCalc(double d_lat, double d_lng, double g_lat, double g_lng, double *dis, double *dir)
{
	double pi = 3.14159265359;
	double lat1 = d_lat * pi / 180;
	double lat2 = g_lat * pi / 180;
	double lng1 = d_lng * pi / 180;
	double lng2 = g_lng * pi / 180;
	double earth_radius = 6378.137;
	*dis = earth_radius * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lng2 - lng1));
	*dir = 90 - (180 / pi * (atan2((cos(lat1) * tan(lat2) - sin(lat1) * cos(lng2 - lng1)), sin(lng2 - lng1))));
	if (*dir < 0)
	{
		*dir += 360;
	}
}
double disCalc(double d_lat, double d_lng, double g_lat, double g_lng)
{
	double pi = 3.14159265359;
	double lat1 = d_lat * pi / 180;
	double lat2 = g_lat * pi / 180;
	double lng1 = d_lng * pi / 180;
	double lng2 = g_lng * pi / 180;
	double earth_radius = 6378.137;
	double dis = earth_radius * acos(sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(lng2 - lng1));
	return dis;
}
void moveDirCalc(double direction, double compass_point, double *move_direction)
{
	if (direction > compass_point)
	{
		double A = direction - compass_point;
		double B = 360 - A;
		if (A > B)
		{
			*move_direction = -B;
		}
		else
		{
			*move_direction = A;
		}
	}
	else
	{
		double A = direction - compass_point;
		double B = 360 - A;
		if (A > B)
		{
			*move_direction = B;
		}
		else
		{
			*move_direction = A;
		}
	}
}

void moveDirCalc2(double direction, double compass_point, double *move_direction)
{
	if (direction > compass_point)
	{
		double A = direction - compass_point;
		double B = 360 - A;
		if (A > B)
		{
			*move_direction = -B;
		}
		else
		{
			*move_direction = A;
		}
	}
	else
	{
		double A = direction - compass_point;
		double B = 360 + A;
		if (abs(A) > B)
		{
			*move_direction = B;
		}
		else
		{
			*move_direction = A;
		}
	}
}

void app_center(double g_lat, double g_lng)
{
	while (1)
	{

		getData(&latitude, &longnitude, &degree);
		dirDisCalc(latitude, longnitude, g_lat, g_lng, &distance, &direction);
		moveDirCalc2(direction, degree, &move_direction);

		if (move_direction < 10 && move_direction > -10)
		{
			if (distance < 0.003)
			{
				setMotorPower(SLOW_PROG_SPEED, SLOW_PROG_SPEED, false);
			}
			else
			{
				setMotorPower(FAST_PROG_SPEED, FAST_PROG_SPEED, false);
			}
		}
		else if (move_direction < 45 && move_direction > 0)
		{
			setMotorPower(ONE_SIDE_ROTATION_SPEED, 0, false);
		}
		else if (move_direction > -45 && move_direction < 0)
		{
			setMotorPower(0, ONE_SIDE_ROTATION_SPEED, false);
		}
		else if (move_direction < 0)
		{
			setMotorPower(-COUNTER_ROTATION_SPEED, COUNTER_ROTATION_SPEED, false);
		}
		else
		{
			setMotorPower(COUNTER_ROTATION_SPEED, -COUNTER_ROTATION_SPEED, false);
		}

		Serial.print("app:lat=");
		Serial.println(latitude, 10);
		Serial.print("app:lng=");
		Serial.println(longnitude, 10);
		Serial.print("app:distance=");
		Serial.println(distance, 10);
		Serial.print("app:move_direction=");
		Serial.println(move_direction, 10);
		if (distance < 0.0003)
		{
			setMotorPower(0, 0, true);
			digitalWrite(BUZ, HIGH);
			delay(3000);
			digitalWrite(BUZ, LOW);

			break;
		}
	}
}
void app_pi(double g_lat, double g_lng)
{
	while (1)
	{

		getData(&latitude, &longnitude, &degree);
		dirDisCalc(latitude, longnitude, g_lat, g_lng, &distance, &direction);
		moveDirCalc2(direction, degree, &move_direction);

		if (move_direction < 10 && move_direction > -10)
		{
			if (distance < 0.0012)
			{
				setMotorPower(SLOW_PROG_SPEED, SLOW_PROG_SPEED, false);
			}
			else
			{
				setMotorPower(FAST_PROG_SPEED, FAST_PROG_SPEED, false);
			}
		}
		else if (move_direction < 45 && move_direction > 0)
		{
			setMotorPower(ONE_SIDE_ROTATION_SPEED, 0, false);
		}
		else if (move_direction > -45 && move_direction < 0)
		{
			setMotorPower(0, ONE_SIDE_ROTATION_SPEED, false);
		}
		else if (move_direction < 0)
		{
			setMotorPower(-COUNTER_ROTATION_SPEED, COUNTER_ROTATION_SPEED, false);
		}
		else
		{
			setMotorPower(COUNTER_ROTATION_SPEED, -COUNTER_ROTATION_SPEED, false);
		}

		Serial.print("app:lat=");
		Serial.println(latitude, 10);
		Serial.print("app:lng=");
		Serial.println(longnitude, 10);
		Serial.print("app:distance=");
		Serial.println(distance, 10);
		Serial.print("app:move_direction=");
		Serial.println(move_direction, 10);
		if (distance < 0.002)
		{
			setMotorPower(0, 0, true);
			break;
		}
	}
}

void lotate_pi(double p_lat, double p_lng, double next_p_lat, double next_p_lng, int clock)
{
	double other_dist;
	double tmp;
	bool over = false;
	bool cl = false;
	while (1)
	{
		getData(&latitude, &longnitude, &degree);
		dirDisCalc(latitude, longnitude, p_lat, p_lng, &distance, &direction);
		moveDirCalc2(direction, degree, &move_direction);
		Serial.print("lotate:distance=");
		Serial.println(distance, 10);
		// distance degree

		if (clock == 0)
		{
			if (distance > 0.0010)
			{
				move_direction -= 50;
			}
			else if (distance < 0.0007)
			{
				move_direction -= 130;
			}
			else
			{
				move_direction -= 80;
			}
		}
		else
		{
			if (distance > 0.0010)
			{
				move_direction += 50;
			}
			else if (distance < 0.0007)
			{
				move_direction += 130;
			}
			else
			{
				move_direction += 80;
			}
		}
		if (move_direction < -180)
		{
			move_direction += 360;
		}
		else if (move_direction > 180)
		{
			move_direction -= 360;
		}
		Serial.print("lotate:move_direction=");
		Serial.println(move_direction, 10);
		if (move_direction < 20 && move_direction > -20)
		{
			setMotorPower(SLOW_PROG_SPEED, SLOW_PROG_SPEED, false);
		}
		else if (move_direction < 0)
		{
			setMotorPower(-COUNTER_ROTATION_SPEED, COUNTER_ROTATION_SPEED, false);
		}
		else
		{
			setMotorPower(COUNTER_ROTATION_SPEED, -COUNTER_ROTATION_SPEED, false);
		}

		dirDisCalc(latitude, longnitude, next_p_lat, next_p_lng, &other_dist, &tmp);
		if (over == true && other_dist < 0.01935 && !cl)
		{
			cl = true;
			break;
		}
		if (!over && other_dist > 0.0203)
		{
			over = true;
			digitalWrite(BUZ, HIGH);
			delay(200);
			digitalWrite(BUZ, LOW);
		}
	}

	getData(&latitude, &longnitude, &degree);
	dirDisCalc(latitude, longnitude, p_lat, p_lng, &distance, &direction);
	float progress_degree = direction;
	if (clock == 0)
	{
		progress_degree -= 60;
	}
	else
	{
		progress_degree += 60;
	}
	if (progress_degree > 360)
	{
		progress_degree -= 360;
	}
	else if (progress_degree < 0)
	{
		move_direction += 360;
	}
	// moveDirCalc2(direction,degree,&move_direction);

	while (1)
	{
		getData(&latitude, &longnitude, &degree);
		dirDisCalc(latitude, longnitude, p_lat, p_lng, &distance, &direction);
		moveDirCalc2(progress_degree, degree, &move_direction);
		if (move_direction < 10 && move_direction > -10)
		{
			setMotorPower(SLOW_PROG_SPEED, SLOW_PROG_SPEED, false);
		}
		else if (move_direction < 45 && move_direction > 0)
		{
			setMotorPower(ONE_SIDE_ROTATION_SPEED, 0, false);
		}
		else if (move_direction > -45 && move_direction < 0)
		{
			setMotorPower(0, ONE_SIDE_ROTATION_SPEED, false);
		}
		else if (move_direction < 0)
		{
			setMotorPower(-COUNTER_ROTATION_SPEED, COUNTER_ROTATION_SPEED, false);
		}
		else
		{
			setMotorPower(COUNTER_ROTATION_SPEED, -COUNTER_ROTATION_SPEED, false);
		}
		if (distance > 0.0015)
		{
			break;
		}
	}
}

void writeEEPROM(int deviceaddress, unsigned int eeaddress, byte data)
{
	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8));	 // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.write(data);
	Wire.endTransmission();

	delay(5);
}

byte readEEPROM(int deviceaddress, unsigned int eeaddress)
{
	byte rdata = 0xFF;

	Wire.beginTransmission(deviceaddress);
	Wire.write((int)(eeaddress >> 8));	 // MSB
	Wire.write((int)(eeaddress & 0xFF)); // LSB
	Wire.endTransmission();

	Wire.requestFrom(deviceaddress, 1);

	if (Wire.available())
		rdata = Wire.read();

	return rdata;
}
