#include <stdio.h>
#include <math.h>
void dirDisCalc(double lat1,double lng1,double lat2,double lng2,double *dis,double *dir){
	double pi = 3.14159265359;
	lat1=lat1*pi/180;
	lat2=lat2*pi/180;
	lng1=lng1*pi/180;
	lng2=lng2*pi/180;
	double earth_radius=6378.137;
	*dis = earth_radius * acos(sin(lat1)*sin(lat2)+cos(lat1)*cos(lat2)*cos(lng2-lng1));
	*dir = 90-(180/pi*(atan2((cos(lat1)*tan(lat2)-sin(lat1)*cos(lng2-lng1)),sin(lng2-lng1))));

}

int main(void){
	double dis,dir;

	dirDisCalc(245.423545,140,34,135,&dis,&dir);
	printf("distance:%lf,direction:%lf\n",dis,dir);
	return 0;
}