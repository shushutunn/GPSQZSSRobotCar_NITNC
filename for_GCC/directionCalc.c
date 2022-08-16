#include <stdio.h>
#include <math.h>
double distance;
double direction;
void directionCalc(double lat1,double lng1,double lat2,double lng2){
	double pi = 3.14159265359;
	lat1=lat1*pi/180;
	lat2=lat2*pi/180;
	lng1=lng1*pi/180;
	lng2=lng2*pi/180;
	double earth_radius=6378.137;
	distance = earth_radius * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lng2-lng1));
	direction = (180/pi*atan(sin(lng2-lng1)/(cos(lat1)*tan(lat2)-sin(lat1)*cos(lng2-lng1))));

}
int main(void){
	directionCalc(36,140,34,135);
	printf("distance:%lf,direction:%lf\n",distance,direction);
	return 0;
}