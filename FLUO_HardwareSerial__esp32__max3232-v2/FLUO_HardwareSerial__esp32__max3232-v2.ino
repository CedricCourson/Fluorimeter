/*
 * Fonctionne ok
 */



#include <HardwareSerial.h>

#define RXD2 16 //25
#define TXD2 17  //26

int pinFluo =27;
int StartStock = 0;

HardwareSerial Serial1(1); // si pin 25/26 choisit alors on sera sur le port Serial2(2) ... à modifier partout dans le programme

String fluochain; 
String datafluo;
int c;
const char marqueurDeFin = '\r';

 
void setup() {

  pinMode(pinFluo, OUTPUT);          // pin fluo
  digitalWrite(pinFluo, HIGH);        // pin fluo off         

  // Note the format for setting a serial port is as follows: Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial.begin(115200);
  //Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial Txd is on pin: "+String(TX));
  Serial.println("Serial Rxd is on pin: "+String(RX));

  delay(3000); 
}



void loop() { 

  //mes_fluo();
  //delay(1000);
  //fluochain = "";

  
  mes_fluo();
  Serial.print("datafluo : ");Serial.println(datafluo);


  //Serial.print("datafluo : "); Serial.println(datafluo);
  //Serial.println("-----------------------------");
 
 delay(3238);
}

void mes_fluo(){
  while(Serial1.available()){
    c = Serial1.read();
    if(StartStock==1){
        switch (c) {
          case marqueurDeFin:
          //Serial.print("fluochain : "); Serial.println(fluochain);
          datafluo = fluochain;
          fluochain="";
          StartStock=0;
          default:
          fluochain += char(c);
        }
    }
    if(c==marqueurDeFin){
      StartStock = 1;
    }
  }
}
