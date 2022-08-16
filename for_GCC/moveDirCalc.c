#include <stdio.h>
#include <math.h>

void move_DirCalc(double direction,double compass_point,double *move_direction){
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
int main(void){
	double move_direction;
	move_DirCalc(245.423545,0.0,&move_direction);
	printf("%lf",move_direction);
	return 0;
}