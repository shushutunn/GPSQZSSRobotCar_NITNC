#include <Wire.h>
void setup() {
    // serial
    Serial.begin(115200);
    Serial.print("i2c test start\r\n");    

    // I2C
    Wire.begin();  // SDA:19,SCL:21 Switch Science Espr developer　 
    Wire.beginTransmission(0x42);
    Wire.write(0xFF);
    Wire.endTransmission(false);
}

void loop() {
    char buff[17];
    Wire.requestFrom(0x42, 16, true);
    int bytes = Wire.available();
    if ( bytes > 0 ){
        for( int i=0; i < bytes; i++ ){
          buff[i] = Wire.read();
        }
     buff[bytes] = '\0';
     if(buff[0]!  ='?') 
        Serial.print( buff );
    }
}
