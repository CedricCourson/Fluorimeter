/*
    ---------- FLUORIMETER ------------------
    www.openoceanography.org
    Fluorimetre open source pour le projet OPEN LAGOON.

    Cet instrument mesure :
    Chlorophyle
    Turbidité
    Température eau de mer
    Salinité eau de mer (coductivité)
    Pression
    + température interne du boitier

   Etat: 1.8 = fonctionne ok, mais bug sur la fluo via DF-i2c-Uart
   ajout GPS
   Calcul Salinité
   Mesure Fluo via module I2c Uart (avec Option pour uart via esp32) + ajout des capa
   Ajout fichier config
   Céreation d'un fichier meta

   Reste a faire :
   - gérer les fichier par date
   - régler le pb de fiabilité avec module i2c Uart, au bout de quelque cycle (2/3 min) les valeur affiche 999 999 999 9999 ....
   - gérer format chain fluo (virgule + le retours a la ligne de la trilux induit un saut de ligne entre chaque ligne de mesure du CSV)
   - harmoniser certaine variable dans ce code et avec SensOcean, enlever les variables superflu
   - eventuellement ajouter la lecture des infos battery dans une fonction
*/



// ---------------------   PARAMETRES MODIFIABLE DU PROGRAM    -----------------------------------
// Version et numero de serie
String fichier_config = "/config.txt";   // nom du fichier de configuration
char versoft[] = "1.82";                 // version du code
int delay_affichage_ecran=2;             // en seconde
int delay_i2c=100;                       // en ms (temps qui suit l'ouverture  ou la fermeture d'une transmission i2c) pas forcément utile
int delay_lec=100;                        // en ms (temps de pause entre la lecture d'un capteur puis un autre)

// --------------------     FIN DES PARAMETRES MODIFIABLES     -----------------------------------




// Library
#include <GxEPD.h>                      // Epaper Screen                          https://github.com/ZinggJM/GxEPD by  Zingg Jean-Marc 
//#include <GxGDEW042T2/GxGDEW042T2.h>    // Epaper Screen 4.2" b/w
#include <GxGDEH029A1/GxGDEH029A1.h>    // Epaper Screen 2.9" b/w
#include <Fonts/FreeMonoBold9pt7b.h>    // font for epaper sreen
#include <Fonts/FreeMonoBold12pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold18pt7b.h>   // font for epaper sreen
#include <Fonts/FreeMonoBold24pt7b.h>   // font for epaper sreen
#include <GxIO/GxIO_SPI/GxIO_SPI.h>     // epaper sceeen
#include <GxIO/GxIO.h>                  // epaper screen
#include <Wire.h>                       // enable I2C.
#include <DS3231.h>                     // Pour horloge RTC                       https://github.com/BollinFee/DS3231   created by Eric Ayars and modified by John Hubert
#include "SPI.h"                        // pour connection ecran bus SPI
#include <SD.h>                         // pour carte SD                         
#include "MS5837.h"                     // pour capteur de pression blue robotics https://github.com/bluerobotics/BlueRobotics_MS5837_Library   
#include <HardwareSerial.h>             // pour capteur serie fluo
#include <SparkFunBQ27441.h>            // Batterie fuel gauge lipo babysister    https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library
#include <OneWire.h>                    // pour capteur ds18b20
#include <DallasTemperature.h>          // pour capteur ds18b20                   https://github.com/milesburton/Arduino-Temperature-Control-Library
#include <DFRobot_IICSerial.h>          // pour interface I2C UART DFROBOT        https://github.com/DFRobot/DFRobot_IICSerial
#include <TinyGPS++.h>                  // Gps                                    https://github.com/mikalhart/TinyGPSPlus


//SPI pin definitions pour ecran epaper
GxIO_Class io(SPI, /*CS=5*/ 0, /*DC=*/ 13, /*RST=*/ 10); // arbitrary selection of 17, 16  //SS remplacé par 0
GxEPD_Class display(io, /*RST=*/ 10, /*BUSY=*/ 4); // arbitrary selection of (16), 4

// définition pour les fonts ecran
const GFXfont* f1 = &FreeMonoBold9pt7b;
const GFXfont* f2 = &FreeMonoBold12pt7b;
const GFXfont* f3 = &FreeMonoBold18pt7b;
const GFXfont* f4 = &FreeMonoBold24pt7b;

//déclaration pour horloge
DS3231 Clock;
bool Century = false;
bool h12;
bool PM;
//byte ADay, AHour, AMinute, ASecond, ABits;
//bool ADy, A12h, Apm;
//byte year, month, date, DoW, hour, minute, second;
String second, minute, hour, date, month, year;
String datenum, timenum;                   // pour format de date en 1 seule écriture
String datetime;

// Set BATTERY_CAPACITY to the design capacity of your battery.
unsigned int BATTERY_CAPACITY = 3400; // e.g. 3400mAh battery
unsigned int soc ;
unsigned int volts;

// definition pour GPS
TinyGPSPlus gps;                           
HardwareSerial SerialGPS(2);  
int gpspin=14;
unsigned long t0;
int gps_tempo=0;


// variable fichier pour SD
String datachain = "";                   // chaine de donnée texte de mesure

//déclaration pour la gestion de fichier sur la carte SD et fichier config
int cspin_SD=5;
String filename, filename_temp, str_index, filetrans;
int ind;                  // index vérification de fichier
//fichier config
String id_logger, user, number_measures, delay_batch, gps_timeOut, fluo_timeOut, fluo_delay_chauffe, mode_screen, bat_capacity ,clef_test;
File confFile;
// default value
int gps_time_out=900;  //secondes
int fluo_time_out=3;   //secondes

//déclaration pour capteur de pression
MS5837 sensor_bar30;
float  wat_pressure, wat_temp, wat_depth, alt;


// ---- déclaration pour capteur de fluo
// si branchement direct sur port serie de l'esp : (dans ce cas les pin R1out/T1in du max3232 sont branché au pin 16/17 de l'esp  R1out => 16, T1in=> 17)
//#define RXD1 16 //25 (si SerialGPS)
//#define TXD1 17 //26 (si SerialGPS)
//HardwareSerial Serial1(1); // si 25/26 alors SerialGPS(2), et modifier tout les Serial1 par SerialGPS. 

// avec module I2c Uart df robot (dans ce cas les pin R1out/T1in du max3232 sont branché au pin du module Dfrobot : R1out => R, T1in=> T) 
/** Mise en place de la ligne I2C->UART.
    ATTENTION A L'ADRESSAGE => IA1=0 IA0=1 . Un autre adressage peut venir en conflit avec les autres devices I2C
    Changer les interrupteurs DIP sur la carte DFROBOT
*/
DFRobot_IICSerial iicSerial1(Wire, /*subUartChannel =*/SUBUART_CHANNEL_1,/*IA1 = */0,/*IA0 = */1);//UART1 de l'interface I2C to UART

String fluochain;
String datafluo;
int c;
const char marqueurDeFin = '\r';
int pinFluo = 27;
int delay_chauffe = 10; // delay de mise en route de la trilux en s
int StartStock = 0;

String chan0="Chl";
String chan1="Nph";
String chan2="Phyco";
String chan3="Valim";
String chan4="Vref";

// definition pour la fonction deepsleep de l'ESP32
#define uS_TO_S_FACTOR 1000000                // Conversion factor for micro seconds to seconds
RTC_DATA_ATTR int bootCount = 0;              // utile pour enregistrer un compteur dans la memoire rtc de l'ULP pour un compteur permettant un suivi entre chaque veille
RTC_DATA_ATTR int filenumber = 0;             // nom de fichier pour le garder entre l'initialisation (1er boot) et les mesures (tout les autres boots)
RTC_DATA_ATTR unsigned long batch_number = 0; //numéro de lot de mesure de puis le démarrage
int TIME_TO_SLEEP = 900;                      // Durée d'endormissement entre 2 cycles complets de mesures (in seconds) par défault 900 = 15min
int nbrMes = 30;                              // nombre de mesure dans un cycle, par défault 30

// adress I2c des capteurs
//#define rtcadrr   104
//#define bar30adrr 118
//#define lipoadrr  85
//#define fluoadrr1 48
//#define fluoadrr2 49
//#define fluoadrr3 50
//#define fluoadrr4 51
//#define fluoadrr5 52
//#define fluoadrr6 53
//#define fluoadrr7 54
//#define fluoadrr8 55 

// definition pour les carte Atlas
#define ecAddress 100
//#define rtdAdress 102
//byte rtdCode=0;                     // Used to hold the I2C response code.
//byte rtdInChar=0;                   // Used as a 1 byte buffer to store in bound bytes from the RTD Circuit.
//char rtdData[20];                   // We make a 20 byte character array to hold incoming data from the RTD circuit.
//int rtdDelay = 600;                 // Used to change the delay needed depending on the command sent to the EZO Class RTD Circuit. 600 by default. It is the correct amount of time for the circuit to complete its instruction.
byte ecCode = 0;                    // Used to hold the I2C response code.
byte ecInChar = 0;                  // Used as a 1 byte buffer to store in bound bytes from the EC Circuit.
char ecData[48];                    // We make a 48 byte character array to hold incoming data from the EC circuit.
int ecDelay = 600;                  // Used to change the delay needed depending on the command sent to the EZO Class EC Circuit. 600 par defaut. It is the correct amount of time for the circuit to complete its instruction.
char *ec;                           // Char pointer used in string parsing.
char *tds;                          // Char pointer used in string parsing.
char *sal;                          // Char pointer used in string parsing.
char *sg;                           // Char pointer used in string parsing.
int ecpin = 12;                     // pin for turn on/off the EC card

// déclaration initiale des variables de la fonction cal_sal (Aminot A., Kérouel R. (2004), Hydrologie des écosystèmes marins. Paramètres et analyses. Cf pages 74-78)
int Rp = 1;
float aa[] = {0.0080, -0.1692, 25.3851, 14.0941, -7.0261, 2.7081};
float bb[] = {0.0005, -0.0056, -0.0066, -0.0375, 0.0636, -0.0144};
float cc[] = {0.6766097, 0.0200564, 0.0001104259, -6.9698E-07, 1.0031E-09};
float k = 0.0162;
float rt, R_t, Salinity, modA, modB;

//déclaration pour capteur de température interne ds18b20
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor_ds18b20(&oneWire);
float tempint;

int pin_dfuart=12;



void setup() {


  // -------------------------------------------------------------------------------------------------------------------
  // ------------------        HERE IS NEEDED THE PARAMETER FOR THE ENTIER PROGRAMM   ----------------------------------
  // -------------------------------------------------------------------------------------------------------------------

  //RESERVE DATA space in buffer
  datafluo.reserve(32);
  datachain.reserve(256);
 
  //INITS
  Serial.begin(115200);                        // communication PC
  //Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1); // communication serie avec la sonde de fluo uniquement si fluo sur UART de l'esp32
  SerialGPS.begin(9600, SERIAL_8N1, 25, 26);     // communication serie avec GPS
  Wire.begin();                                // for I2C communication
  //Wire.setClock(100000);
  display.init();                              // enable display Epaper
  delay(500); //Take some time to open up the Serial Monitor and enable all things

  pinMode(pinFluo, OUTPUT);          // define pin fluo
  pinMode(gpspin, OUTPUT);            // define pin gps
  pinMode(pin_dfuart, OUTPUT);
  digitalWrite(pinFluo, LOW);        // turn off the pin fluo
  digitalWrite(gpspin, LOW);          // turn off the gps
  digitalWrite(pin_dfuart, LOW);


  // Test et initialisation des componsant blocquant (carte SD, batterie, pression)
  setupBQ27441();                              // for Lipo Babysister sparkfun lipo fuelgauge
  test_sd();                                   // test SD
  test_bar;                                    // test capteur de pression


  // on lit le fichier de configuration pour en extraire les valeurs 
  lecture_config();
    // on redifinie les variables de mise en veille à partir de la lecture du fichier config.txt
    TIME_TO_SLEEP=delay_batch.toInt();         // en s
    nbrMes=number_measures.toInt();            
    gps_time_out=gps_timeOut.toInt()*1000;    // résultat en s
    fluo_time_out=fluo_timeOut.toInt()*1000;  // résulat en s
    mode_screen=mode_screen.toInt();        
    BATTERY_CAPACITY=bat_capacity.toInt();    // en mAh   
    delay_chauffe=fluo_delay_chauffe.toInt();  // en s


  if (bootCount == 0) //Run this only the first time
  {
    // -----------------------------------------------------------------------------------------------------------
    // --------------        HERE IS ONLY THE INTRODUCION           ----------------------------------------------
    // -----------------------------------------------------------------------------------------------------------
   
    // affiche texte d'intro
    if(mode_screen=1){
          affiche_intro();
          delay(delay_affichage_ecran*1000);
    }
    

//    //création nom de fichier pour logger les infos :  
//          // création du nom de fichier et écriture sur la carte SD en automatisant la vérification de fichiers existant.
//          filename = ""; filename += "/data";
//          str_index = String(1000);
//          filename_temp = filename + str_index + ".csv";              //1er fichier du jour
//          int ind = 0;
//      
//          while (SD.exists(filename_temp)) {                        // vérifie l'existantce de fichier dans un boucle en indexant le numéro de fichier
//            ind++;
//            str_index = String(1000 + ind);
//            filename_temp = filename + str_index + ".csv";
//          }
//          filename = filename_temp;                                   // création du nom de fichier final
//      
//          filenumber = str_index.toInt();                             //convertion du nom de fichier en int pour transfert en memoire RTC
//      
//          // fin de création de fichier


    //Create and Write the 1st ligne of the CSV file
        datachain += "Batch Number"; datachain += ";";
        datachain += "Date & heure"; datachain += ";";
        datachain += "Bat %"; datachain += ";" ; datachain += "Bat mV"; datachain += ";";
        datachain += "Intern Temp (C)"; datachain += ";";
        datachain += "Profondeur(m)"; datachain += ";";   datachain += "Temperature mer (C)"; datachain += ";";
        datachain += "Conductivity(ms/cm)"; datachain += ";"; datachain += "Salinity"; datachain += ";";
        datachain += chan0; datachain += ";";datachain += chan1; datachain += ";";datachain += chan2; datachain += ";";datachain += chan3; datachain += ";";datachain += chan4; datachain += ";";

        // print the datachain on the serial port
        Serial.println("Format de la chaine enregistrée : ");
        Serial.println(datachain);
    

        // on créer le nom du fichier
        lecture_rtc();
        filename = "/"+String(id_logger)+"_"+datenum+".csv";
        
        // save the datachain on the SD card
        File dataFile = SD.open(filename, FILE_APPEND);
        if (dataFile) {                                        // if the file is available, write to it:
          dataFile.println(datachain);
          dataFile.close();
          Serial.println("Fichier créer avec succes");
          Serial.print("Filename : "); Serial.println(filename);
        }
        else {                                                 // if the file isn't open, pop up an error:
          Serial.println("error opening file");
          if(mode_screen=1) errormessage_sd();
          delay(3000);
        }

    delay(500);

    // affiche le message d'infos (nom de fichier, delay batch etc...)
    if(mode_screen=1){
      message1();
    }

    bootCount = bootCount + 1; // change the bootcount number to skip the initial step after wake up

  } else
  {
    // ----------------------------------------------------------------------------
    // ----------------------- HERE IS THE MAIN LOOP ------------------------------
    // ----------------------------------------------------------------------------

    // on se reveille et démarre un "lot" de mesure
      batch_number ++ ;                 // Number for this measures batch
      Serial.print("------ BATCH NUMBER ---------"); Serial.println(batch_number); 

    // on récupére le nom de fichier pour l'enregistrement des data
      lecture_rtc();
      delay(500);
      filename="/"+String(id_logger)+"_"+datenum+".csv";
      Serial.print("File name : "); Serial.println(filename);
    
    // Reveillez l'alim de la trilux et la laisser chauffer le temps qu'on exécute le reste
      Serial.println("on reveille la trilux, et on la laisse chauffer : ");Serial.println(delay_chauffe);Serial.println(" s");
      digitalWrite(pinFluo, HIGH);               // turn on the trilux sensor
      delay(delay_chauffe * 1000);               // give time to the trilux sensor to start (min 10 seconds)


    // on lit les sensors 1 par 1 et on stocke les résultats dans les variables
    
    // Initialize pressure Sensor ------ACHANGER
//    Wire.beginTransmission(bar30adrr);
//    delay(delay_i2c);
        while (!sensor_bar30.init()) {                                            // Returns true if initialization was successful
          Serial.println("Init failed!");                                     // We can't continue with the rest of the program unless we can initialize the sensor
          Serial.println("Are SDA/SCL connected correctly?");
          Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
          Serial.println("\n\n\n");
          delay(5000);
        }
        sensor_bar30.setModel(MS5837::MS5837_30BA);
        sensor_bar30.setFluidDensity(1029); // kg/m^3 (1029 for seawater, 997 for freshwater)
//    Wire.endTransmission(bar30adrr);
    
    delay(delay_lec);


  
    for (int i = 0; i < nbrMes; i++) {
      Serial.print("--- Serie n=° "); Serial.println(i + 1);

      // lecture des différents capteurs
      lecture_rtc();            // lecture de l'horloge rtc
      delay(delay_lec);
      read_battery();           // lecture des valeurs de la batterie
      delay(delay_lec);
      mes_temp_int();           // temperature interne du boitier
      delay(delay_lec);
      mesure_pressure();        // measure water pressure/temp
      delay(delay_lec);
      mesureEC();               // mesure de la conductivité EC de l'eau de mer
      delay(delay_lec);
      mes_fluo();               // on lit la chaine et on l'enregistre dans la variable datafluo
      delay(delay_lec);

      // Read battery stats from the BQ27441-G1A -- ACHANGER
//      Wire.beginTransmission(bar30adrr);
//      delay(delay_i2c);
      unsigned int soc = lipo.soc();  // Read state-of-charge (%)
      unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
//      Wire.endTransmission(bar30adrr);
//      Wire.endTransmission();
      delay(500);

      //calcul de la salinité
      float temp = wat_temp;
      float cond = atof(ecData)/1000;
      cal_sal(temp, cond);      

      // Création de la chaine à enregistrer sur la carte SD
      String datachain = "";
      datachain += batch_number; datachain += ";";
      datachain += datetime; datachain += ";";
      datachain += soc; datachain += ";"; datachain += volts ; datachain += ";";
      datachain += tempint; datachain += ";";
      datachain += wat_depth; datachain += ";"; datachain += wat_temp; datachain += ";";
      datachain += ecData; datachain += ";";datachain += Salinity; datachain += ";";
      datachain += fluochain; //datachain += ";";

      // enregistrement sur la carte SD
      File dataFile = SD.open(filename, FILE_APPEND);
      if (dataFile) {                                        // if the file is available, write to it:
        dataFile.println(datachain);
        dataFile.close();
        Serial.println("ok : Datachain saved on SD card !");
      }
      else {                                                 // if the file isn't open, pop up an error:
        Serial.println("error opening file");
        errormessage_sd();
        delay(3000);
      }

      // Affichage port serie des datas
      Serial.println("Datachain : "); Serial.println(datachain); //affichage de la chaine complete sur le port serie
      Serial.println(" ");
      Serial.println("------------------------");
      Serial.println(" ");
    }
    // fin des mesures


    // On éteint les sensors pour la mise ne veille de l'esp32
    veille_ezos();                            // mise en veille de la carte Atlas
    digitalWrite(pinFluo, LOW);               // turn on the trilux sensor


    // On affiche sur l'ecran la deniére serie du batch
    if(mode_screen=1){
        display.setRotation(3);
        display.fillScreen(GxEPD_WHITE);
    
        display.setTextColor(GxEPD_BLACK);
        display.setFont(f1);
        display.setCursor(5, 10); display.print("Depth :"); display.print(wat_depth); display.print(" m");
        display.setCursor(5, 30); display.print("Sea T :"); display.print(wat_temp); display.print(" deg C");
        display.setCursor(5, 50); display.print("Sea S :"); display.print(Salinity);
        display.setCursor(5, 70); display.print("Fluo  :"); display.print(fluochain);
    
        //cadre
        //display.fillRect(147, 0, 2, 90, GxEPD_BLACK);
        display.fillRect(0, 94, 296, 2, GxEPD_BLACK);

        unsigned int soc = lipo.soc();  // Read state-of-charge (%)  // si on met pas ca ici, l'affichage battery indique 0, on relit donc la valeur de la battery
    
        //Last update
        display.setFont(f1);
        display.setCursor(0, 110); display.print(datetime);
        //display.setCursor(0,110); display.print(date);display.print("/");display.print(month);display.print("/");display.print(year);  //date
        //display.setCursor(123,110);display.print(hour);display.print("h");display.print(minute);                                       //heure
        display.setCursor(210, 110); display.print("Bat:"); display.print(soc); display.print("%");
        display.setCursor(0, 124); display.print("Temp interne : "); display.print(tempint); display.print(" deg C");
        //      display.setCursor(0, 125); display.print("Lat:");display.print(gps.location.lat(), 5);                                                     //lattitude
        //      display.setCursor(148,125); display.print("Lng:");display.print(gps.location.lng(), 4);                                                   //longitude
        display.update();
    }
    
   
    delay (500);
  }


  // Affichage du temps d'endormissement en fonction du temps d'exécution de lecture des capteurs 
  Serial.print("Temp de dodo = "); Serial.print(((TIME_TO_SLEEP * uS_TO_S_FACTOR)-millis()*1000)/uS_TO_S_FACTOR); Serial.println(" Secondes");
  Serial.print("Temps de fonctionnement de ce lot = "); Serial.print(millis()); Serial.print(" ou "); Serial.println(millis()/1000);
    
  // endormissement esp32
  Serial.println("Going to sleep");
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR-millis()*1000);  
  esp_deep_sleep_start();
}


void loop() {
  //never use because of the sleeping of the princess esp32
}



/*
   -----------------------------------------------------------------------------------------------------------------------------------------------------
   -------------------- FUNCTIONS CALLED IN THE PROGAM -------------------------------------------------------------------------------------------------
   -----------------------------------------------------------------------------------------------------------------------------------------------------
*/



// LES FONCTIONS D'AFFICHAGE


void affiche_intro() {                                 // texte intro à l'allumage, juste pour faire jolie et afficher éventuellement les versions du programme
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  // subtitle
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f1);
  display.setCursor(5, 10  );
  display.println("OpenSourceOceanograpy");

  //titre
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("Fluorimeter");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("Num Serie :"); display.println(id_logger);
  display.setCursor(30, 110);
  display.print("Ver Soft  :"); display.print(versoft);

  display.update();
}


void message1() {                                 // texte intro à l'allumage, juste pour faire jolie et afficher éventuellement les versions du programme
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  // subtitle
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f1);
  display.setCursor(0, 20); display.println("Startup file & SD : ok");
  display.setCursor(0, 40); display.print("File name :"); display.print(filename);

  display.setCursor(0, 90); display.print("Acquisition every "); display.print(TIME_TO_SLEEP / 60); display.print(" min");
  display.setCursor(0, 110); display.print("Cycle by acquisition :"); display.print(nbrMes);

  display.update();
}




void errormessage_sd() {                                      // message d'erreur en cas d'impossibilité de lire la carte SD

  Serial.println("Error : error with sd card");
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("  ERROR ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("SD Card ");

  display.update();
}

void errormessage_bar() {                                      // message d'erreur en cas d'impossibilité de lire la carte SD

  Serial.println("Error : error with sd card");
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("  ERROR ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("BAR30 Sensor");

  display.update();
}

void errormessage_battery() {                                      // message d'erreur en cas d'impossibilité de lire la carte SD

  Serial.println("Error : error BATTERY");
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);

  //titre
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f3);
  display.setCursor(25, 55  );
  display.println("  ERROR ");

  //version
  display.setFont(f1);
  display.setCursor(30, 90);
  display.print("BATTERY ");

  display.update();
}


void affiche_searchfix(){                 //message de recherche de GPS
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f2);
  display.setCursor(5, 55);
  display.println("GPS: wait for fix ...");
 
  display.update();
}


void affiche_fixok(){                     //message GPS ok et affiche delay entre les series de mesures
  display.setRotation(3);
  display.fillScreen(GxEPD_WHITE);
  
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f2);
  display.setCursor(0, 20);
  display.println("GPS : fix ok ");
  display.update();
}


// -------------------------------------------------------- LES FONCTIONS DE GESTION DES SENSORS

void mesure_pressure() {                              // fonction mesure de la pression et température sur le capteur blue robotics


//  Wire.beginTransmission(bar30adrr);
//  delay(delay_i2c);

  // Update pressure and temperature readings
  sensor_bar30.read();

  wat_pressure = sensor_bar30.pressure();
  wat_temp = sensor_bar30.temperature();
  wat_depth = sensor_bar30.depth();
  alt = sensor_bar30.altitude();

  Serial.println("--- Pressure Sensor :");
  Serial.print("Pressure: ");
  Serial.print(wat_pressure);
  Serial.println(" mbar");

  Serial.print("Temperature: ");
  Serial.print(wat_temp);
  Serial.println(" deg C");

  Serial.print("Depth: ");
  Serial.print(wat_depth);
  Serial.println(" m");

  Serial.print("Altitude: ");
  Serial.print(alt);
  Serial.println(" m above mean sea level");
  //Serial.println(" ");


//  Wire.endTransmission(bar30adrr); 
//  delay(delay_i2c);
}

// Test pressure sensor
void test_bar(){
//  Wire.beginTransmission(bar30adrr);
//  delay(delay_i2c);
      if(!sensor_bar30.init()) {                                            // Returns true if initialization was successful
      Serial.println("Init failed!");                                     // We can't continue with the rest of the program unless we can initialize the sensor
      Serial.println("Are SDA/SCL connected correctly?");
      Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
      Serial.println("\n\n\n");
      errormessage_bar();
    }
//  Wire.endTransmission(bar30adrr);
  
}



// FLUO si branchement sur DFrobot I2c Uart, utiliser cette fonction
void mes_fluo() {

  Serial.println("-- Mesure Fluo --");
  
  // Turn in df uart module
  digitalWrite(pin_dfuart, HIGH);            
  delay(1000);

//  Wire.beginTransmission(fluoadrr1);
//  Wire.beginTransmission(fluoadrr2);
//  Wire.beginTransmission(fluoadrr3);
//  Wire.beginTransmission(fluoadrr4);
//  Wire.beginTransmission(fluoadrr5);
//  Wire.beginTransmission(fluoadrr6);  
//  Wire.beginTransmission(fluoadrr7);
//  Wire.beginTransmission(fluoadrr8);
//  delay(delay_i2c);
  
  //Init UART
  iicSerial1.begin(/*baud = */9600);
  delay(100);


  //Flush data
  // Note : le composant a un buffer intégré qu'il faut vider avant d'avoir une trame en temps réel
  Serial.println("Flush data ");
  while (iicSerial1.available())
    iicSerial1.read();

  char c = ' ';

  // Retirer les donnees incompletes
  Serial.println("remove incomplete data ...");
  unsigned long temps0=millis();
  unsigned int time_out=5000;
  while (c != '\n' && (millis()-temps0)<time_out) {
    if (iicSerial1.available())
      c  = iicSerial1.read();
  }

  //Fetch correct data
  Serial.print("Read data: ");
  datafluo = "";
  c = ' '; // Reinit process
  temps0=millis();
  time_out=5000;
  while (c != '\n' && (millis()-temps0)<time_out) {
    if (iicSerial1.available()) {
      c  = iicSerial1.read();
      Serial.print(c);
      datafluo += c;
    }
  }

  //END UART
  iicSerial1.end(); // Pour ne pas squatter la ligne I2C
  delay(100);

  //turnoff trilux and DFUart module
  //digitalWrite(pinFluo, LOW);
  digitalWrite(pin_dfuart, LOW);

//  // end i2c 
//  delay(100);
//  Wire.endTransmission(fluoadrr1);
//  Wire.endTransmission(fluoadrr2);
//  Wire.endTransmission(fluoadrr3);
//  Wire.endTransmission(fluoadrr4);
//  Wire.endTransmission(fluoadrr5);
//  Wire.endTransmission(fluoadrr6);  
//  Wire.endTransmission(fluoadrr7);
//  Wire.endTransmission(fluoadrr8);
//  delay(delay_i2c);

  //Serial.print("datafluo : "); Serial.println(datafluo); // juste pour voir la valeur de datafluo

  //----------------- parsing and filtering  data ---------------------
  // Dans la suite on filtre et on éclate les valeurs de datachain issu du capteur de fluo et on les formats dans une chaine "fluchain", et c'est fluochain qu'on ajoute à la datachain total

  fluochain = "";
  
  //Serial.println(" Parsing data :");
  int fluo_length= datafluo.length();
  char fluochar_temp[fluo_length];
  char fluochar[fluo_length];
  
  datafluo.toCharArray(fluochar_temp, fluo_length);  // converti les string datafluo en tableau char (necessaire pour la fonction strtok)

  // filtration de valeur de texte V (on profite que c'est un table pour extraire les valeurs que l'on ne veut pas...
  for (int i=0; i<=fluo_length; i++){
    char temp = fluochar_temp[i];
    if (temp != 'V'){                   // on filtre la valeur 'V' et ? et on peut ajouter autant de condition que de filtrage voulu
      fluochar[i] = temp;
    }
    else{
      fluochar[i] = ' ';                 // si on rajoute pas ce else, alors la chaine est coupé aprés le 1er filtrage de V (les valeurs suivante ne sont pas ajouté)
    }
  }

  Serial.print(" Filtered fluochar "); Serial.println(fluochar);

  char * pnt;                                  //création d'un pointeur pour l'enregistrement des tokens
  pnt = strtok(fluochar,",");         // parse le tableau de char en utilisant le délimiter , chaque parse est un token

  // lit chaque token, l'affiche et l'ajouter dans la string fluochain
  //int index=0;  // on commence par zero, car la trilux commence sur le channel 0
  while(pnt != NULL){
    //Serial.println(pnt);
    String val_temp = ""; val_temp += pnt;           // enregistrement de la valeur du pointeur dans une string
    val_temp.trim();                                 // on supprime les espaces de début et de fin
    fluochain += val_temp; fluochain += ";";         // on ajoute le token à fluochain
    //index++;                                         
    pnt=strtok(NULL, ",");                           // obligatoire car liée au fonction de strtok qui a besoin d'un pointeur définit a NULL pour afficher le token suivant 
  }

  free(pnt);          // libére l'espace mémoire des token de strtok 

  Serial.println("--- Fluo Sensor :");
  Serial.print(" Fluochain : "); Serial.println(fluochain);

  // ---------------- end parsing function ----------------------
  // end function fluo
  
}


//// FLUO si branchement sur uart 1 de l'esp, utiliser cette fonction :
//void mes_fluo(){
//  while(Serial1.available()){
//    c = Serial1.read();
//    if(StartStock==1){
//        switch (c) {
//          case marqueurDeFin:
//          //Serial.print("fluochain : "); Serial.println(fluochain);
//          datafluo = fluochain;
//          fluochain="";
//          StartStock=0;
//          default:
//          if(c!=marqueurDeFin){fluochain += char(c);}
//        }
//    }
//    if(c==marqueurDeFin){
//      StartStock = 1;
//    }
//  }
//
//  Serial.println("--- Fluo Sensor :");
//  Serial.print("datafluo : "); Serial.println(datafluo); // juste pour voir la valeur de datafluo
//}



void mesureEC() {
  Serial.println("--- EC Sensor :");
  Wire.beginTransmission(ecAddress);   // Call the circuit by its ID number.
  Wire.write('r');                     // r for reading sensor
  Wire.endTransmission();              // End the I2C data transmission.
  delay(ecDelay);                      // Reading time needing, 600 by default
  Wire.requestFrom(ecAddress, 48, 1);  // Call the circuit and request 48 bytes (this is more than we need)
  ecCode = Wire.read();                // The first byte is the response code, we read this separately.
  byte i = 0;                          // Counter used for EC_data array.
  while (Wire.available()) {           // Are there bytes to receive.
    ecInChar = Wire.read();            // Receive a byte.
    ecData[i] = ecInChar;              // Load this byte into our array.
    i += 1;                            // Incur the counter for the array element.
    if (ecInChar == 0) {               // If we see that we have been sent a null command.
      i = 0;                           // Reset the counter i to 0.
      Wire.endTransmission();          // End the I2C data transmission.
      break;                           // Exit the while loop.
    }
  }
  switch (ecCode) {                    // Switch case based on what the response code is.
    case 1:                            // Decimal 1.
      Serial.println("EC Success");    // Means the command was successful.
      break;                           // Exits the switch case.
    case 2:                            // Decimal 2.
      Serial.println("EC Failed");     // Means the command has failed.
      break;                           // Exits the switch case.
    case 254:                          // Decimal 254.
      Serial.println("EC Pending");    // Means the command has not yet been finished calculating.
      break;                           // Exits the switch case.
    case 255:                          // Decimal 255.
      Serial.println("EC No Data");    // Means there is no further data to send.
      break;                           // Exits the switch case.
  }
  ec = strtok(ecData, ",");
  tds = strtok(NULL, ",");
  sal = strtok(NULL, ",");
  sg = strtok(NULL, ",");    // Let's pars the string at each comma.

  Serial.print("EC value : "); Serial.println(ecData);
  //  Serial.print("EC:");                //we now print each value we parsed separately.
  //  Serial.println(ec);                 //this is the EC value.
  //  Serial.print("TDS:");               //we now print each value we parsed separately.
  //  Serial.println(tds);                //this is the TDS value.
  //  Serial.print("SAL:");               //we now print each value we parsed separately.
  //  Serial.println(sal);                //this is the salinity value.
  //  Serial.print("SG:");                //we now print each value we parsed separately.
  //  Serial.println(sg);                 //this is the specific gravity.
}

void veille_ezos(){                          //mise en veille du capteur aprés utilisation (la fonction "r" permettra de reveiller et lire le capteur).
//       //veille temperature
//        delay(800);
//        Wire.beginTransmission(rtdAddress);    // Call the circuit by its ID number.  
//        Wire.write("Sleep");                   // Transmit the command that was sent through the serial port.
//        Wire.endTransmission();                // End the I2C data transmission. 
       //veille EC
        delay(100);
        Wire.beginTransmission(ecAddress);    // Call the circuit by its ID number.  
        Wire.write("Sleep");                  // Transmit the command that was sent through the serial port.
        Wire.endTransmission();               // End the I2C data transmission. 
}




void cal_sal(float t, float C){         // fonction sinplifié à intégrer dans un code
  // calcul intermédiaire
  int i;
  rt=0;                                 // réinitilisation entre chaque boucle
  for (i=0; i<5; i++){                  // calcul de rt
    rt += cc[i]*pow(t,float(i));
  }

  R_t= C /(42.914*rt);                  // calcul de R_t

  modA =0; modB=0;                      // réinitilisation entre chaque boucle
  for (i=0; i<6;i++){                   // cal modA et modB
    modA += aa[i]*pow(R_t,float(i/2));
    modB += bb[i]*pow(R_t,float(i/2));
  }

  //cal salinité
  Salinity = modA+((t-15)/(1+k*(t-15)))*modB;
  Serial.print("Salinité = "); Serial.println(Salinity,4);
}



void mes_temp_int() {   // température interne du boitier pour info DS18b20
  sensor_ds18b20.begin();
  delay(500);
  sensor_ds18b20.requestTemperatures(); // Send the command to get temperatures
  tempint = sensor_ds18b20.getTempCByIndex(0);
  Serial.println("--- Interne Temp sensor :");
  Serial.print("Temperature : ");
  Serial.println(tempint, 3);   // We use the function ByIndex, and as an example get the temperature from the first sensor only.

}




// ----------------------------------------------------------   LES FONCTION DES CAPTEURS ET COMPOSANTS POUR Datalog



void lecture_rtc() {                          // fonction lecture de l'horloge RTC et ecriture dans les variables utilent
//  Wire.beginTransmission(rtcadrr);
//  delay(100);
  
  int sec = Clock.getSecond();
  if (sec < 10) {
    second = String(0) + String(sec);
  }
  else {
    second = sec;
  }
  int minu = Clock.getMinute();
  if (minu < 10) {
    minute = String(0) + String(minu);
  }
  else {
    minute = minu;
  }
  int heure = Clock.getHour(h12, PM);
  if (heure < 10) {
    hour = String(0) + String(heure);
  }
  else {
    hour = heure;
  }
  int jour = Clock.getDate();
  if (jour < 10) {
    date = String(0) + String(jour);
  }
  else {
    date = jour;
  }
  int mois = Clock.getMonth(Century);
  if (mois < 10) {
    month = String(0) + String(mois);
  }
  else {
    month = mois;
  }
  year = Clock.getYear();
  datenum = ""; datenum += year; datenum += month; datenum += date;
  timenum = ""; timenum += hour; timenum += minute; timenum += second;
  datetime = ""; datetime += date; datetime += "/"; datetime += month; datetime += "/20"; datetime += year; datetime += " "; datetime += hour; datetime += ":"; datetime += minute; datetime += ":"; datetime += second;

  Serial.println("--- RTC values :");
  Serial.print("Date :"); Serial.println(datenum);
  Serial.print("Time :"); Serial.println(timenum);
  Serial.print("DateTime"); Serial.println(datetime);
  //Serial.print(" ");

//  Wire.endTransmission(rtcadrr);
//  delay(100);  
}

void read_battery(){
//  Wire.beginTransmission(lipoadrr);
//  delay(delay_i2c);
      // Read battery stats from the BQ27441-G1A
      unsigned int soc = lipo.soc();  // Read state-of-charge (%)
      unsigned int volts = lipo.voltage(); // Read battery voltage (mV)
      Serial.println("--- Battery sensor : ");
      Serial.print(" Charge % :"); Serial.println(soc);
      Serial.print(" Tension mV : "); Serial.println(volts);


//  Wire.beginTransmission(lipoadrr); 
}


void setupBQ27441(void)
{
//  Wire.beginTransmission(lipoadrr);
//  delay(delay_i2c);
  
  // Use lipo.begin() to initialize the BQ27441-G1A and confirm that it's
  // connected and communicating.
  if (!lipo.begin()) // begin() will return true if communication is successful
  {
    // If communication fails, print an error message and loop forever.
    Serial.println("Error: Unable to communicate with BQ27441.");
    Serial.println("  Check wiring and try again.");
    Serial.println("  (Battery must be plugged into Battery Babysitter!)");
    errormessage_battery();
    while (1) ;
  }
  Serial.println("Connected to BQ27441!");

  // Uset lipo.setCapacity(BATTERY_CAPACITY) to set the design capacity
  // of your battery.
  lipo.setCapacity(BATTERY_CAPACITY);

//  Wire.endTransmission(lipoadrr);
//  delay(delay_i2c);
}


static void smartDelay(unsigned long ms)      //fonction pour GPS         
{
  unsigned long start = millis();
  do
  {
    while (SerialGPS.available())
      gps.encode(SerialGPS.read());
  } while (millis() - start < ms);
}


void test_sd(){
  Serial.print("-- Initializing SD card... :");   
  if (!SD.begin(cspin_SD)) {                      // // see if the card is present and can be initialized, ajouter ici chipSelect ou 5 pour la pin 5 par default
  Serial.println("Card failed, or not present");
  errormessage_sd();
  // don't do anything more:
  while(1);
  }
  Serial.println("card initialized.");    
}


void lecture_config(){
  confFile = SD.open(fichier_config, FILE_READ);

  char phrase[200];
  byte index = 0;
  char x=0;
  String reste = "";
  int k=0;
  
  if (confFile) {
    
    while (confFile.available()) {
      x = confFile.read();
      if (x!=10){
        if (x!=13) {
          phrase[index] = x;
          index++;
        } else {
          if (index != 0) {
            reste = phrase;
            reste = reste.substring(0,index);
            index=0;
            // suppression des commentaires
            k = reste.indexOf(";");
            if (k!=0) {
              if (k!=-1) {
                reste = reste.substring(0,k);
              }
              // supprime les espaces au début et à la fin
              reste.trim();
              //
              // extrait les valeurs pour les placer dans les variables du programme
              //
              if (reste.indexOf('=') >0) {
                // signe égal trouvé
                String clef = reste.substring(0,reste.indexOf('='));
                String valeur = reste.substring(reste.indexOf('=')+1);
                if (clef == "id_logger") id_logger = valeur;
                if (clef == "User") user = valeur;
                if (clef == "delay_batch") delay_batch = valeur;
                if (clef == "number_measures") number_measures = valeur;
                if (clef == "gps_time_out") gps_timeOut = valeur;
                if (clef == "fluo_time_out") fluo_timeOut = valeur;
                if (clef == "fluo_delay_chauffe") fluo_delay_chauffe = valeur;
                if (clef == "mode_screen") mode_screen = valeur;                         
                if (clef == "clef_test") clef_test = valeur;
              }
            }
          }          
        }
      }
     }
    // close the file:
    confFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening "+fichier_config);
  }
   
}


//Création fichier metadonnées
void meta_create(){           

       Serial.println("-- Create meta file for tacking GPS : ");
       //allumer le GPS pour la recherche de satellite
          affiche_searchfix();                    // ecran recherche GPS fix
          Serial.println("Searching GPS :");
          digitalWrite(gpspin, HIGH);             // GPS on
          delay(1000);                            // pour que le GPS se reveille tranquillement
          t0=millis();                            // temporisation pour attendre le fix du GPS et lecture du GPS
          int gps_timer=t0+gps_time_out;
          while(millis()<gps_timer){            // attends le delay imposer par config.txt, puis passe à la suite fix ou pas fix
             //Read the gps
             smartDelay(1000);  
             if (millis() > 5000 && gps.charsProcessed() < 10)
                Serial.println(F("No GPS data received: check wiring")); 
             // if fix ok, break the while loop   
             if(gps.location.isUpdated()){
              affiche_fixok();         // ecran fix ok, maintenant les mesures seront affichés tout les x mins
              Serial.println("Ok GPS fix");
              break;
             }
             delay(500);
          }
          affiche_fixok();         // ecran fix ok, maintenant les mesures seront affichés tout les x mins
          delay(1000);

        //Créer le fichier de metadonnées 
          File metaFile = SD.open("/meta.txt", FILE_APPEND);
          if (metaFile) {                                        // if the file is available, write to it:
            metaFile.print("Idlogger :"); metaFile.println(id_logger);
            metaFile.print("Lat logger :"); metaFile.println(String(gps.location.lat(),6));
            metaFile.print("Lng logger :"); metaFile.println(String(gps.location.lng(),6));
            metaFile.print("Start date :"); metaFile.println(gps.date.value());
            metaFile.print("Start time :"); metaFile.println(gps.time.value());
            metaFile.println(" -------------------------- ");
            metaFile.close();
            Serial.println("ok - file created");
          }
          else {                                                 // if the file isn't open, pop up an error:
            Serial.println("error opening file");
            errormessage_sd();
            delay(3000);
          }       
}

// créer un fichier à la date du jour

//void datafile_create(){
//      lecture_rtc();
//  
//      // création du nom de fichier et écriture sur la carte SD en automatisant la vérification de fichiers existant.
//        filename =""; filename +="/"; filename += datenum;
//        filename_temp = filename + ".csv";     //1er fichier du jour
//        int ind=1;
////        while(SD.exists(filename_temp)){        // vérifie l'existantce de fichier dans un boucle en indexant le numéro de fichier
////          str_index=String(ind);
////          filename_temp = filename + str_index + ".csv";
////          ind++;
////        }
//  
//        filename = filename_temp;   // création du nom de fichier final
//
//      //convertion du nom de fichier en int pour transfert en memoire RTC
//      filetrans = datenum + str_index;
//      filenumber = filetrans.toInt();
//}
