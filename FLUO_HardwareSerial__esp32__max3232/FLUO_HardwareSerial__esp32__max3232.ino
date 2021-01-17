/*
 * Fonctionne mais pb avec longueur de la fluochain qui est trés variable.... donc fonctionne pas bien ....
 */



#include <HardwareSerial.h>

#define RXD2 25
#define TXD2 26

HardwareSerial Serial2(2); 

String fluochain;
int c;
const char marqueurDeFin = '\r';

 
void setup() {
  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));

  delay(1000); 
}



void loop() { 
 mes_fluo();

 Serial.print("fluochain : "); Serial.println(fluochain);
 Serial.println("--------");
 delay(5000);

  
}

void mes_fluo(){
  fluochain="";
  int readfluo = 1;
  while(Serial2.available() && readfluo==1){
  c = Serial2.read();
  if(c != -1){
    switch (c) {
      case marqueurDeFin:
      fluochain += ";";
      //Serial.println(fluochain);
      readfluo=0;
      default:
      fluochain += char(c);
    }
   }
  }
  
}
