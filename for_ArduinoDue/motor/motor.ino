#define  L_MOTOR_INA 12
#define  L_MOTOR_INB 10
#define  L_MOTOR_PWM 11
#define  R_MOTOR_INA 4
#define  R_MOTOR_INB 2
#define  R_MOTOR_PWM 3
#define  BTN_1 9



void setup() {
  // put your setup code here, to run once:
  pinMode(L_MOTOR_INA,OUTPUT);
  pinMode(L_MOTOR_INB,OUTPUT);
  pinMode(R_MOTOR_INA,OUTPUT);
  pinMode(R_MOTOR_INB,OUTPUT);
  pinMode(BTN_1,INPUT_PULLUP);
	while(digitalRead(BTN_1) == HIGH);
}

void loop() {


  setMotorPower(500,500,false);
  delay(500);
  setMotorPower(0,0,true);
  delay(500);
  setMotorPower(-500,-500,false);
  delay(500);
  setMotorPower(0,0,false);
  delay(500);
  setMotorPower(500,-500,false);
  delay(500);
  setMotorPower(0,0,false);
  delay(500);
  setMotorPower(-500,500,false);
  delay(500);
  setMotorPower(0,0,false);
  delay(500);
  setMotorPower(500,-500,false);
  delay(500);
  setMotorPower(0,0,false);
  delay(500);
  while(1){
    
  }
} 

void setMotorPower(int l_power,int r_power,boolean m_break){

  if(m_break == true){
    digitalWrite(L_MOTOR_INA,HIGH);
    digitalWrite(L_MOTOR_INB,HIGH);
    analogWrite(L_MOTOR_PWM,0);
    digitalWrite(R_MOTOR_INA,HIGH);
    digitalWrite(R_MOTOR_INB,HIGH);
    analogWrite(R_MOTOR_PWM,0);
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
